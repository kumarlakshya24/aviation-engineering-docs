# SOP-BMS-001: Pre-Flight Battery System Check

**Document ID**: SOP-BMS-001  
**Version**: 3.2  
**Effective Date**: 2024-11-15  
**Classification**: Safety Critical (synthetic example)  
**Approval**: VP Safety & Certification, Halcyon Aero

## 1. Purpose

This procedure defines the mandatory pre-flight battery system verification
for the Halcyon Aero H1 eVTOL aircraft. Compliance is required before every flight
under the H1 type certification basis and operating limitations.

The pre-flight limits below are deliberately tighter than the runtime protection limits in
`battery-management/bms_monitor.py` (HA-BMS-SPEC-2024-003 Table 3.1).

## 2. Scope

Applies to all flight operations personnel, maintenance technicians, and
ground crew performing pre-flight checks on the H1 battery pack (6 modules,
396 cells, NMC 811 chemistry).

## 3. Prerequisites

- [ ] Aircraft power-on complete (SOP-GND-001)
- [ ] Ground support equipment (GSE) connected
- [ ] BMS telemetry link active on GCS

## 4. Procedure

### 4.1 Visual Inspection (5 min)

1. Inspect all 6 battery module enclosures for physical damage
2. Verify coolant line connections: no leaks or condensation
3. Check module mounting bolts torque markers (visual alignment)
4. Verify thermal management inlet/outlet vents are unobstructed

### 4.2 BMS Telemetry Verification (3 min)

1. On the GCS Battery tab, verify ALL 396 cells report telemetry
2. Confirm pack state shows `READY` (not `FAULT` or `PROTECTION`)
3. Record the following in the flight log:

| Parameter | Acceptable Range | Action if Out-of-Range |
|-----------|-----------------|----------------------|
| SOC | 80-100% | Charge to 90%+ before flight |
| SOH | >85% | Flag for maintenance review |
| Min Cell Voltage | >3.6 V | Do not fly, investigate |
| Max Cell Voltage | <4.18 V | Wait for balancing |
| Cell Voltage Delta | <50 mV | If >50mV, run balance cycle |
| Min Cell Temp | >5 degC | Allow warmup if cold-soaked |
| Max Cell Temp | <40 degC | Allow cooldown |
| Pack Current (idle) | <2 A | If >2A, investigate parasitic draw |

### 4.3 Protection System Test (2 min)

1. Command BMS self-test via GCS: `BMS > Diagnostics > Self-Test`
2. Verify all protection circuits respond:
   - Over-voltage protection: **PASS**
   - Under-voltage protection: **PASS**
   - Over-temperature protection: **PASS**
   - Over-current protection: **PASS**
   - Cell isolation relay: **PASS**
3. If any test fails: **NO-GO**. Initiate SOP-BMS-005 (Fault Isolation)

### 4.4 Sign-Off

1. Both pilot and ground crew sign the battery check section of the flight log
2. GCS automatically timestamps the check completion
3. Battery check validity: **2 hours** from completion

## 5. Emergency Procedures

If a battery anomaly is detected during pre-flight:
- **Voltage anomaly**: Follow SOP-BMS-003 (Cell Voltage Management)
- **Temperature anomaly**: Follow SOP-BMS-004 (Thermal Runaway Prevention)
- **Communication loss**: Follow SOP-BMS-006 (BMS Communication Recovery)

## 6. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 3.2 | 2024-11-15 | Power Systems | Added cell delta voltage check |
| 3.1 | 2024-09-01 | Power Systems | Updated temp ranges for winter ops |
| 3.0 | 2024-06-15 | Safety & Certification | Aligned with certification basis revision 4 |
| 2.0 | 2024-01-10 | Power Systems | Added protection system self-test |
| 1.0 | 2023-08-01 | Power Systems | Initial release |
