import pytest
from bms_monitor import BMSMonitor, CellReading, CellState, PackState


def _cell(cell_id=0, v=3.8, t=25.0, r=20.0, module_id=0) -> CellReading:
    return CellReading(module_id=module_id, cell_id=cell_id, voltage=v, temperature=t,
                       internal_resistance=r, cycle_count=100)


@pytest.mark.parametrize(
    ("kwargs", "expected"),
    [
        ({}, CellState.NOMINAL),
        ({"v": 2.95}, CellState.WARNING),
        ({"v": 4.16}, CellState.WARNING),
        ({"t": 50.0}, CellState.WARNING),
        ({"r": 60.0}, CellState.DEGRADED),
        ({"v": 2.8}, CellState.FAULT),
        ({"v": 4.2}, CellState.FAULT),
        ({"t": 60.0}, CellState.FAULT),
        ({"t": -25.0}, CellState.FAULT),
    ],
)
def test_cell_classification(kwargs, expected):
    assert BMSMonitor().update_cell(_cell(**kwargs)) == expected


def test_empty_pack_reports_fault():
    assert BMSMonitor().get_pack_status(0.0).state == PackState.FAULT


def test_pack_state_follows_current_direction():
    bms = BMSMonitor()
    for i in range(10):
        bms.update_cell(_cell(cell_id=i))
    assert bms.get_pack_status(50.0).state == PackState.DISCHARGING
    assert bms.get_pack_status(-20.0).state == PackState.CHARGING
    assert bms.get_pack_status(0.0).state == PackState.READY
    assert bms.get_pack_status(450.0).state == PackState.PROTECTION


def test_more_than_three_faulted_cells_takes_pack_offline():
    bms = BMSMonitor()
    for i in range(10):
        bms.update_cell(_cell(cell_id=i, v=2.7 if i < 4 else 3.8))
    status = bms.get_pack_status(10.0)
    assert status.cells_in_fault == 4
    assert status.state == PackState.FAULT


def test_repeated_fault_is_counted_once():
    bms = BMSMonitor()
    for _ in range(3):
        bms.update_cell(_cell(v=2.7))
    assert bms.get_pack_status(0.0).cells_in_fault == 1


@pytest.mark.parametrize(("v", "soc"), [(4.2, 100.0), (2.5, 0.0), (3.475, 50.0)])
def test_voltage_to_soc(v, soc):
    assert BMSMonitor._voltage_to_soc(v) == pytest.approx(soc)
