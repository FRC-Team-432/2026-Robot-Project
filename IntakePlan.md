## Intake Plan

### Button Bindings
- **Left trigger (held):** spin intake motor forward — collect game piece
- **Left bumper (held):** spin intake motor in reverse — eject game piece
- **Right bumper (held):** FaceTagCommand CW search (unchanged)

### Implementation
- Motor: TalonFX (Kraken/Falcon), Phoenix 6, default CAN bus
- CAN ID: **placeholder 40** — update `IntakeConstants.INTAKE_CAN_ID` once motor is wired
- Control: open-loop DutyCycleOut (no PID/MotionMagic needed)
- Neutral mode: Brake (stops quickly so game pieces don't slip)
- Forward speed: `INTAKE_SPEED = 0.8` (80% duty cycle)
- Reverse speed: `EJECT_SPEED = 0.5` (50% duty cycle)

### Files Created/Modified
| File | Change |
|------|--------|
| `src/.../constants/IntakeConstants.java` | New — CAN ID placeholder + speed constants |
| `src/.../subsystems/intake/Intake.java` | New — TalonFX subsystem with intake() and eject() commands |
| `src/.../RobotContainer.java` | Added intake field + left trigger / left bumper bindings; removed left bumper FaceTagCommand |

### Outstanding TODOs
- [ ] Set real CAN ID in `IntakeConstants.INTAKE_CAN_ID` once motor is wired
- [ ] Tune `INTAKE_SPEED` and `EJECT_SPEED` on the real robot
- [ ] Verify motor direction (may need to negate speeds if intake spins backwards)
