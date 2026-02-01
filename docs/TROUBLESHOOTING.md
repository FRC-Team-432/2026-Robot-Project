# Troubleshooting Guide

**Solutions to common problems you'll encounter.**

---

## Table of Contents

1. [Build Errors](#build-errors)
2. [Deployment Errors](#deployment-errors)
3. [Runtime Errors](#runtime-errors)
4. [Motor/CAN Issues](#motorcan-issues)
5. [Vision/Camera Issues](#visioncamera-issues)
6. [Drivetrain Issues](#drivetrain-issues)
7. [Controller Issues](#controller-issues)

---

## Build Errors

### "JAVA_HOME is not set"

**Problem:** Gradle can't find Java.

**Solution:**
1. Make sure WPILib is installed (it includes Java)
2. Open project in VS Code with WPILib extension
3. Use WPILib's terminal (Ctrl+Shift+P → "WPILib: Open WPILib Terminal")

### "Could not resolve dependencies"

**Problem:** Can't download libraries.

**Solutions:**
```bash
# 1. Check internet connection

# 2. Refresh dependencies
./gradlew --refresh-dependencies build

# 3. Clear gradle cache
rm -rf ~/.gradle/caches
./gradlew build
```

### "Compilation failed" with Java errors

**Problem:** Syntax error in code.

**Solution:** Read the error message carefully!
```
/src/main/java/frc/robot/RobotContainer.java:45: error: ';' expected
    private final Arm arm = new Arm()
                                     ^
```
- Line 45 is missing a semicolon
- Fix the syntax error and rebuild

### "Could not find class"

**Problem:** Missing import or wrong class name.

**Solution:**
1. Check the import statements at top of file
2. Make sure class name matches file name exactly
3. Verify the package name is correct

### "Permission denied" (Mac/Linux)

**Problem:** gradlew isn't executable.

**Solution:**
```bash
chmod +x gradlew
./gradlew build
```

---

## Deployment Errors

### "Could not find roborio"

**Problem:** Can't connect to RoboRIO.

**Solutions:**

1. **Check robot is on**
   - Main breaker switched on?
   - Wait 30+ seconds for RoboRIO to boot

2. **Check WiFi connection**
   - Connected to robot's network?
   - Try pinging: `ping roboRIO-432-FRC.local`

3. **Check Ethernet (if direct connect)**
   - Cable plugged in to RoboRIO Ethernet port?
   - Try IP directly: `ping 10.4.32.2`

4. **Try USB connection**
   - Connect USB-B cable to RoboRIO
   - IP becomes: `172.22.11.2`

### "Deploy failed: Target is not available"

**Problem:** RoboRIO found but deploy failed.

**Solutions:**
1. Re-image the RoboRIO with latest firmware
2. Check RoboRIO has correct team number
3. Make sure no other code is being deployed simultaneously

### "Robot code not starting"

**Problem:** Code deployed but not running.

**Solutions:**
1. Check Driver Station for errors
2. Look at RoboRIO console output
3. Redeploy with verbose output:
   ```bash
   ./gradlew deploy --info
   ```

---

## Runtime Errors

### "Command scheduler loop overrun"

**Problem:** Robot code taking too long per cycle.

**Solutions:**
1. Check for infinite loops in code
2. Remove `System.out.println()` in frequently called methods
3. Reduce complexity in `periodic()` methods

### "CAN Timeout"

**Problem:** Motor controller not responding.

**Solutions:**
1. Check CAN wiring
2. Verify CAN ID matches code
3. Check for duplicate CAN IDs
4. Verify motor controller has power (lights on?)

### "Null Pointer Exception"

**Problem:** Using an object that wasn't initialized.

**Solution:**
```
java.lang.NullPointerException
    at frc.robot.RobotContainer.configureBindings(RobotContainer.java:150)
```
- Line 150 is using something that's `null`
- Check that all objects are created in the correct order

### "HAL: DriverStation not initialized"

**Problem:** Running on RoboRIO before driver station connected.

**Solution:** This is often normal at startup. Wait for DS to connect.

---

## Motor/CAN Issues

### Motor Not Spinning

**Checklist:**
1. [ ] Motor controller has power (LEDs lit?)
2. [ ] CAN ID in code matches physical device
3. [ ] Motor phase/inversion correct
4. [ ] No CAN wiring issues
5. [ ] Motor not in brake mode preventing movement

**Debugging:**
```java
// Add this to see what's happening
System.out.println("Motor output: " + motor.get());
System.out.println("Motor voltage: " + motor.getMotorVoltage().getValue());
```

### Wrong Motor Direction

**Problem:** Motor spins opposite of expected.

**Solution in code:**
```java
// In subsystem constructor:
motor.setInverted(true);  // or false

// For TalonFX with Phoenix 6:
var config = new TalonFXConfiguration();
config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
motor.getConfigurator().apply(config);
```

### CAN ID Conflicts

**Problem:** Two devices with same ID.

**Symptoms:**
- Erratic behavior
- Multiple motors moving together
- Random disconnections

**Solution:**
1. Use Phoenix Tuner X to scan all devices
2. Change one device's ID to be unique
3. Update code to match new ID

### Motor Flashing Error Codes

**TalonFX LED Colors:**
| Color | Meaning |
|-------|---------|
| Green blink | Normal operation |
| Orange blink | No CAN communication |
| Red blink | Fault condition |
| Off | No power |

Check Phoenix Tuner X for specific fault codes.

---

## Vision/Camera Issues

### "Camera not detected"

**Problem:** Limelight not showing in dashboard.

**Solutions:**
1. Check Limelight has power (green light)
2. Verify network connection
3. Try accessing web interface: `http://10.4.32.11:5801`
4. Check Limelight name matches code

### Vision Lock Not Working

**Problem:** LT held but robot doesn't auto-aim.

**Common causes:**
1. **No AprilTag visible** - Check camera view
2. **Wrong alliance selected** - Press LB or RB
3. **Running in simulation** - Vision doesn't work in sim!

**Debug:**
```java
System.out.println("Tag visible: " + tracker.isTagVisible());
System.out.println("Tag offset: " + tracker.getHorizontalOffset());
```

### AprilTag Not Recognized

**Problem:** Tag visible on camera but not detected.

**Solutions:**
1. Check tag is official FRC AprilTag
2. Verify tag isn't damaged/dirty
3. Adjust Limelight exposure settings
4. Make sure tag is well-lit
5. Check AprilTag pipeline is selected

### Camera Feed Laggy

**Problem:** High latency on camera image.

**Solutions:**
1. Reduce resolution in Limelight settings
2. Increase compression
3. Check network bandwidth
4. Disable unused processing pipelines

---

## Drivetrain Issues

### Robot Drifts to One Side

**Problem:** Robot doesn't drive straight.

**Solutions:**
1. Reset gyro (Start button)
2. Check wheel alignments
3. Verify all swerve modules working
4. Check for mechanical issues (loose wheels, friction)

### Swerve Wheels Out of Alignment

**Problem:** Wheels don't point the same direction.

**Solutions:**
1. Re-zero CANcoders in Phoenix Tuner X
2. Update offset values in TunerConstants.java
3. Check absolute encoder magnets haven't moved

### Robot Spins Unexpectedly

**Problem:** Robot rotates when it shouldn't.

**Solutions:**
1. Check gyro calibration
2. Reset gyro heading
3. Verify field-centric mode is working
4. Check for faulty encoder

### Slow Drivetrain Response

**Problem:** Robot feels sluggish.

**Solutions:**
1. Check slow mode isn't stuck on
2. Verify speed multipliers in DriveConstants
3. Check for mechanical drag
4. Review input curve settings

---

## Controller Issues

### Controller Not Detected

**Problem:** Joystick shows as disconnected.

**Solutions:**
1. Check USB connection
2. Try different USB port
3. Verify driver station sees controller
4. Check controller port number matches code

### Wrong Controller Mapping

**Problem:** Buttons do different things than expected.

**Solutions:**
1. Verify which physical port controller is in
2. Check if it's driver (port 0) or operator (port 1)
3. Review button bindings in RobotContainer

### Controller Drift

**Problem:** Robot moves with joystick centered.

**Solutions:**
1. Increase deadband in DriveConstants:
   ```java
   public static final double JOYSTICK_DEADBAND = 0.15; // was 0.1
   ```
2. Replace worn controller
3. Calibrate controller in Windows/Mac settings

---

## Quick Diagnostics

### First Things to Check

```
□ Robot power on?
□ RoboRIO booted (30+ seconds)?
□ Connected to robot WiFi?
□ Driver Station shows green?
□ Code deployed successfully?
□ Robot enabled?
□ Joysticks connected?
```

### Reading Error Messages

Error messages tell you EXACTLY what's wrong:

```
edu.wpi.first.wpilibj.DriverStation: ERROR  Error at frc.robot.Robot.robotInit(Robot.java:35):
Unhandled exception: java.lang.RuntimeException:
  CAN device timeout on TalonFX CAN ID 31
```

This tells you:
- Error type: CAN device timeout
- Device: TalonFX
- CAN ID: 31
- Location: Robot.java line 35

### Useful Commands

```bash
# Check network connectivity
ping roboRIO-432-FRC.local

# View RoboRIO console output
ssh admin@roboRIO-432-FRC.local
netconsole

# Deploy with verbose output
./gradlew deploy --info

# Clean and rebuild
./gradlew clean build
```

---

## Getting More Help

1. **Check the docs first** - Many answers are in STUDENT_GUIDE.md
2. **Read the error message** - It usually tells you what's wrong
3. **Ask a mentor** - They've seen it all before
4. **Chief Delphi forums** - FRC community help
5. **WPILib Documentation** - Official docs at docs.wpilib.org

---

*FRC Team 432 - The Final Countdown*
