"""Battery Management System Monitor: real-time cell monitoring & protection.

Monitors individual cell voltages, temperatures, and current draw across
the Halcyon Aero H1's 6-module battery pack (396 cells total). Implements protection
logic per HA-BMS-SPEC-2024-003.

Safety-critical: This code runs on the BMS ECU and is subject to DO-178C
DAL-A certification requirements.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import IntEnum

logger = logging.getLogger(__name__)


class CellState(IntEnum):
    """Cell health classification per HA-BMS-SPEC Section 2.1."""
    NOMINAL = 0
    WARNING = 1       # Approaching limits
    DEGRADED = 2      # Performance reduced
    CRITICAL = 3      # Immediate action required
    FAULT = 4         # Cell isolated


class PackState(IntEnum):
    """Overall pack state per HA-BMS-SPEC Section 2.2."""
    READY = 0
    CHARGING = 1
    DISCHARGING = 2
    BALANCING = 3
    PROTECTION = 4    # Protection active, reduced capability
    FAULT = 5         # Pack offline


# Operating limits per HA-BMS-SPEC-2024-003 Table 3.1
CELL_VOLTAGE_MIN = 2.8      # V, hard cutoff
CELL_VOLTAGE_LOW = 3.0      # V, warning threshold
CELL_VOLTAGE_NOMINAL = 3.7  # V, nominal
CELL_VOLTAGE_HIGH = 4.15    # V, warning threshold
CELL_VOLTAGE_MAX = 4.20     # V, hard cutoff

CELL_TEMP_MIN = -20.0       # degC, no discharge below this
CELL_TEMP_LOW = 0.0         # degC, reduced current below this
CELL_TEMP_HIGH = 45.0       # degC, reduced current above this
CELL_TEMP_MAX = 60.0        # degC, emergency shutdown

PACK_CURRENT_MAX = 400.0    # A, continuous discharge limit
PACK_CURRENT_BURST = 600.0  # A, 30s burst limit

# Pack configuration: 6 modules, 66 cells per module (22S3P per module)
CELLS_PER_MODULE = 66
MODULES_PER_PACK = 6
TOTAL_CELLS = CELLS_PER_MODULE * MODULES_PER_PACK  # 396


@dataclass
class CellReading:
    """Individual cell measurement snapshot."""
    module_id: int
    cell_id: int
    voltage: float            # Volts
    temperature: float        # degC
    internal_resistance: float  # mOhm
    cycle_count: int
    state: CellState = CellState.NOMINAL


@dataclass
class PackStatus:
    """Aggregated pack status for flight computer."""
    state: PackState
    total_voltage: float       # V
    current: float             # A (positive = discharge)
    soc_percent: float         # State of charge [0-100]
    soh_percent: float         # State of health [0-100]
    min_cell_voltage: float
    max_cell_voltage: float
    min_cell_temp: float
    max_cell_temp: float
    cells_in_warning: int
    cells_in_fault: int
    estimated_range_nm: float  # Nautical miles remaining
    time_to_empty_min: float   # Minutes at current draw


class BMSMonitor:
    """Real-time Battery Management System monitor.

    Processes cell-level telemetry at 10 Hz and publishes pack status
    to the flight computer via CAN bus (CAN-ID 0x100-0x10F).
    """

    def __init__(self):
        self._cell_readings: dict[tuple[int, int], CellReading] = {}
        self._pack_state = PackState.READY
        self._protection_active = False
        self._fault_cells: set[tuple[int, int]] = set()

    def update_cell(self, reading: CellReading) -> CellState:
        """Process a single cell reading and return its health state.

        Args:
            reading: Fresh cell voltage/temperature measurement.

        Returns:
            Updated cell state classification.
        """
        state = self._classify_cell(reading)
        reading.state = state

        key = (reading.module_id, reading.cell_id)
        self._cell_readings[key] = reading

        if state == CellState.FAULT and key not in self._fault_cells:
            logger.critical(
                "CELL FAULT | module=%d cell=%d V=%.3f T=%.1f",
                reading.module_id, reading.cell_id,
                reading.voltage, reading.temperature,
            )
            self._fault_cells.add(key)

        return state

    def get_pack_status(self, current_draw: float) -> PackStatus:
        """Compute aggregated pack status from all cell readings.

        Args:
            current_draw: Pack-level current in Amps (positive = discharge).

        Returns:
            PackStatus with aggregated metrics for the flight computer.
        """
        if not self._cell_readings:
            return PackStatus(
                state=PackState.FAULT,
                total_voltage=0, current=0, soc_percent=0, soh_percent=0,
                min_cell_voltage=0, max_cell_voltage=0,
                min_cell_temp=0, max_cell_temp=0,
                cells_in_warning=0, cells_in_fault=0,
                estimated_range_nm=0, time_to_empty_min=0,
            )

        voltages = [r.voltage for r in self._cell_readings.values()]
        temps = [r.temperature for r in self._cell_readings.values()]

        warning_count = sum(
            1 for r in self._cell_readings.values()
            if r.state in (CellState.WARNING, CellState.DEGRADED)
        )
        fault_count = len(self._fault_cells)

        # Determine pack state based on protection logic
        pack_state = self._determine_pack_state(voltages, temps, current_draw)

        # SOC estimation (simplified coulomb counting + OCV lookup)
        avg_voltage = sum(voltages) / len(voltages)
        soc = self._voltage_to_soc(avg_voltage)

        # SOH from internal resistance degradation
        resistances = [r.internal_resistance for r in self._cell_readings.values()]
        avg_resistance = sum(resistances) / len(resistances)
        soh = max(0, min(100, 100 - (avg_resistance - 20) * 2))

        # Range estimation: energy remaining / power consumption
        energy_wh = soc / 100 * 22 * 66 * 3.7 * 6  # Simplified pack energy
        power_w = max(abs(current_draw) * sum(voltages) / 66, 1)
        time_to_empty = energy_wh / power_w * 60  # minutes
        range_nm = time_to_empty / 60 * 150 * (soc / 100)  # 150 kt cruise

        return PackStatus(
            state=pack_state,
            total_voltage=sum(voltages) / 3,  # 3P config
            current=current_draw,
            soc_percent=soc,
            soh_percent=soh,
            min_cell_voltage=min(voltages),
            max_cell_voltage=max(voltages),
            min_cell_temp=min(temps),
            max_cell_temp=max(temps),
            cells_in_warning=warning_count,
            cells_in_fault=fault_count,
            estimated_range_nm=range_nm,
            time_to_empty_min=time_to_empty,
        )

    def _classify_cell(self, r: CellReading) -> CellState:
        """Classify cell health based on voltage and temperature limits."""
        if r.voltage <= CELL_VOLTAGE_MIN or r.voltage >= CELL_VOLTAGE_MAX:
            return CellState.FAULT
        if r.temperature >= CELL_TEMP_MAX or r.temperature <= CELL_TEMP_MIN:
            return CellState.FAULT
        if r.voltage <= CELL_VOLTAGE_LOW or r.voltage >= CELL_VOLTAGE_HIGH:
            return CellState.WARNING
        if r.temperature >= CELL_TEMP_HIGH or r.temperature <= CELL_TEMP_LOW:
            return CellState.WARNING
        if r.internal_resistance > 50:  # mOhm, degraded cell
            return CellState.DEGRADED
        return CellState.NOMINAL

    def _determine_pack_state(
        self, voltages: list[float], temps: list[float], current: float
    ) -> PackState:
        """Determine overall pack state from cell data and current draw."""
        if len(self._fault_cells) > 3:
            return PackState.FAULT
        if min(voltages) < CELL_VOLTAGE_LOW or max(temps) > CELL_TEMP_HIGH:
            return PackState.PROTECTION
        if abs(current) > PACK_CURRENT_MAX:
            return PackState.PROTECTION
        if current > 0:
            return PackState.DISCHARGING
        if current < -1:
            return PackState.CHARGING
        return PackState.READY

    @staticmethod
    def _voltage_to_soc(voltage: float) -> float:
        """Approximate SOC from open-circuit voltage (NMC chemistry)."""
        if voltage >= 4.15:
            return 100.0
        if voltage <= 2.8:
            return 0.0
        # Piecewise linear approximation
        return max(0, min(100, (voltage - 2.8) / (4.15 - 2.8) * 100))
