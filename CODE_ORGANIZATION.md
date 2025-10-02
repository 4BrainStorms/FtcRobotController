# Code Organization Guide for FTC Robot Controller

## 📋 Table of Contents
1. [Current Structure Analysis](#current-structure-analysis)
2. [Recommended Package Structure](#recommended-package-structure)
3. [File Organization Guidelines](#file-organization-guidelines)
4. [Naming Conventions](#naming-conventions)
5. [Best Practices](#best-practices)
6. [Migration Guide](#migration-guide)

---

## 🔍 Current Structure Analysis

### ✅ What's Working Well

Your codebase already has some good organizational patterns in place:

**Good subdirectory organization:**
- `auton/` - Autonomous OpModes (13 files)
- `config/` - Configuration classes (4 files)
- `subsystems/` - Robot subsystems (4 files)
- `telemetry/` - Telemetry helpers (2 files)
- `teleop/` - TeleOp modes (1 file)
- `tuning/` - Tuning utilities (4 files)
- `util/` - Utility classes (3 files)
- `vision/` - Vision-related code (1 file)
- `messages/` - Data transfer objects (8 files)
- `debug/` - Debug utilities (1 file)

**Archive folder:**
- `Archive/` - Properly separated old/experimental code (17 files)

### ⚠️ Areas for Improvement

**Root directory clutter (18 files):**
The `org.firstinspires.ftc.teamcode` root package contains too many files that should be organized into subdirectories:

```
teamcode/
├── Drawing.java
├── DriveTeleOpSimple.java
├── DriveTeleOpWithMotor.java
├── DriveTeleOp_Tag24_Pinpoint60in.java
├── DriveTeleopTelemetry.java
├── DualControllerAlign.java
├── GoBildaPinpointDriver.java
├── Localizer.java
├── MecanumDrive.java
├── OTOSLocalizer.java
├── PinpointLocalizer.java
├── RRPG_FieldToRR_Quick.java
├── SensorGoBildaPinpointExample.java
├── SimpleAlignToTag.java
├── TankDrive.java
├── TeleopTuningConfig.java
├── ThreeDeadWheelLocalizer.java
└── TwoDeadWheelLocalizer.java
```

---

## 📁 Recommended Package Structure

Here's the ideal organization for your TeamCode:

```
org.firstinspires.ftc.teamcode/
│
├── 📂 auton/                      # Autonomous OpModes
│   ├── BlueAllianceAutonRR.java
│   ├── RedAllianceAutonRR.java
│   ├── LimelightAutonBaseRR.java
│   ├── AutonomousLogicEnhancer.java
│   ├── RRPaths.java               # Path definitions
│   ├── RRPG_Runner_Center.java
│   └── ...sanity tests
│
├── 📂 teleop/                     # TeleOp/Driver-controlled modes
│   ├── DriveTeleOpSimple.java
│   ├── DriveTeleOpWithMotor.java
│   ├── DriveTeleOp_Tag24_Pinpoint60in.java
│   ├── DriveTeleopTelemetry.java
│   ├── DualControllerAlign.java
│   └── DashboardTeleOpSample.java
│
├── 📂 subsystems/                 # Robot subsystem classes
│   ├── DriveBase.java
│   ├── IntakeBase.java
│   ├── LaunchBase.java
│   └── LimelightBase.java
│
├── 📂 drive/                      # Drive system implementations
│   ├── MecanumDrive.java         # Main mecanum drive class
│   ├── TankDrive.java            # Tank drive class
│   └── Drawing.java              # Drive visualization
│
├── 📂 localization/               # Odometry and localization
│   ├── Localizer.java            # Interface
│   ├── PinpointLocalizer.java
│   ├── OTOSLocalizer.java
│   ├── ThreeDeadWheelLocalizer.java
│   ├── TwoDeadWheelLocalizer.java
│   └── GoBildaPinpointDriver.java
│
├── 📂 vision/                     # Computer vision
│   ├── LLUsbSanity.java
│   ├── SimpleAlignToTag.java
│   └── SensorGoBildaPinpointExample.java
│
├── 📂 config/                     # Configuration classes
│   ├── AllianceTagConfig.java
│   ├── LaunchCalibration.java
│   ├── LaunchZoneToggle.java
│   ├── RobotInitConfig.java
│   └── TeleopTuningConfig.java
│
├── 📂 util/                       # Utility classes
│   ├── PinpointConfig.java
│   ├── StartPoses.java
│   ├── StrafeHealthCheck.java
│   └── RRPG_FieldToRR_Quick.java
│
├── 📂 tuning/                     # Tuning OpModes
│   ├── TuningOpModes.java
│   ├── LocalizationTest.java
│   ├── ManualFeedbackTuner.java
│   └── SplineTest.java
│
├── 📂 telemetry/                  # Telemetry helpers
│   ├── DashboardLayout.java
│   └── LauncherTelemetry.java
│
├── 📂 messages/                   # Data transfer objects
│   ├── DriveCommandMessage.java
│   ├── MecanumCommandMessage.java
│   ├── PoseMessage.java
│   └── ...
│
├── 📂 debug/                      # Debug utilities
│   └── HeadingHoldKpStore.java
│
├── 📂 test/                       # Test and sanity check OpModes
│   ├── PinpointSanity.java
│   ├── RR_FollowerSanity.java
│   ├── RR_AxisProbe.java
│   ├── RR_RawDriveProbe.java
│   ├── AxisSanity.java
│   └── PinpointHeadingCheck.java
│
└── 📂 Archive/                    # Archived/experimental code
    └── (old implementations)
```

---

## 📝 File Organization Guidelines

### 1. **Separate by Function, Not by Type**

**❌ Don't organize by type:**
```
teleop/
  - AllTeleOpModes.java (1000+ lines)
```

**✅ Do organize by function:**
```
teleop/
  - DriveTeleOpSimple.java        (basic driving)
  - DriveTeleOpWithMotor.java     (with manipulator)
  - DashboardTeleOpSample.java    (with telemetry)
```

### 2. **OpMode Placement Rules**

| OpMode Type | Location | Example |
|------------|----------|---------|
| Autonomous programs | `auton/` | `BlueAllianceAutonRR.java` |
| Driver-controlled | `teleop/` | `DriveTeleOpSimple.java` |
| Tuning/calibration | `tuning/` | `ManualFeedbackTuner.java` |
| Testing/debugging | `test/` | `PinpointSanity.java` |

### 3. **Support Class Organization**

| Class Purpose | Location | Example |
|--------------|----------|---------|
| Drive implementations | `drive/` | `MecanumDrive.java` |
| Localizers | `localization/` | `PinpointLocalizer.java` |
| Robot subsystems | `subsystems/` | `IntakeBase.java` |
| Configuration | `config/` | `RobotInitConfig.java` |
| Vision processing | `vision/` | `SimpleAlignToTag.java` |
| Utilities | `util/` | `StartPoses.java` |
| Messages/DTOs | `messages/` | `PoseMessage.java` |

### 4. **What Goes in the Root Package?**

**Only these files (or none at all):**
- `readme.md` - Documentation
- Very large, core files if they don't fit elsewhere (rare)

**Everything else should be in a subdirectory.**

---

## 🏷️ Naming Conventions

### OpMode Naming

Follow FTC SDK conventions with clear, descriptive names:

**Pattern:** `[Purpose][Alliance/Type][Implementation]`

**Examples:**
```java
// Autonomous
@Autonomous(name="Auton: BLUE (LL+RR)", group="AA Auton")
public class BlueAllianceAutonRR extends LinearOpMode { }

@Autonomous(name="Auton: RED (LL+RR)", group="AA Auton")
public class RedAllianceAutonRR extends LinearOpMode { }

// TeleOp
@TeleOp(name="Drive TeleOp (Robot-Centric — Simple)", group="drive")
public class DriveTeleOpSimple extends LinearOpMode { }

@TeleOp(name="Drive TeleOp (Field-Centric w/ Motor)", group="drive")
public class DriveTeleOpWithMotor extends LinearOpMode { }

// Tuning
@Autonomous(name="RR Follower Sanity", group="debug")
public class RR_FollowerSanity extends LinearOpMode { }

@Autonomous(name="Pinpoint Sanity", group="debug")
public class PinpointSanity extends LinearOpMode { }
```

### Class Naming

**OpModes:**
- `BlueAllianceAutonRR` - Alliance-specific autonomous
- `DriveTeleOpSimple` - TeleOp with descriptor
- `PinpointSanity` - Test/sanity check

**Support Classes:**
- `MecanumDrive` - Implementation name
- `PinpointLocalizer` - Type + purpose
- `DriveBase` - Subsystem + "Base"
- `StartPoses` - Plural for collections/utilities

**Avoid:**
- ❌ `RRPG_ActionsRun_Center` (too many underscores)
- ❌ `Tag24FieldApproachRR` (version numbers in name)
- ❌ `DriveTeleOp_Tag24_Pinpoint60in` (too specific)

### Package Naming

Use lowercase, descriptive names:
```
✅ auton, teleop, subsystems, localization
❌ Auton, TeleOp, SubSystems, Localization
```

### Group Naming for Driver Station

Use consistent group names for easy filtering:

```java
// Autonomous modes
@Autonomous(name="...", group="AA Auton")  // "AA" sorts to top

// TeleOp modes
@TeleOp(name="...", group="drive")

// Tuning/Debug
@Autonomous(name="...", group="debug")
@TeleOp(name="...", group="tuning")

// Archive/Experiments
@Autonomous(name="...", group="zz")  // "zz" sorts to bottom
```

---

## ✅ Best Practices

### 1. **One Public Class Per File**

Each `.java` file should contain exactly one public class matching the filename.

```java
// ✅ Good - MecanumDrive.java
public class MecanumDrive { ... }

// ✅ OK - helper classes can be package-private
class DriveLocalizer { ... }

// ❌ Bad - multiple public classes
public class MecanumDrive { ... }
public class TankDrive { ... }
```

### 2. **Use Base Classes for Common Logic**

Create abstract base classes to reduce duplication:

```java
// LimelightAutonBaseRR.java
public abstract class LimelightAutonBaseRR extends LinearOpMode {
    protected abstract boolean isBlueAlliance();
    protected abstract Pose2d getStartPose();
    protected abstract Action buildSequence(MecanumDrive drive, Pose2d startPose);
    
    @Override
    public void runOpMode() {
        // Common initialization and execution logic
    }
}

// BlueAllianceAutonRR.java - simple subclass
public class BlueAllianceAutonRR extends LimelightAutonBaseRR {
    @Override protected boolean isBlueAlliance() { return true; }
    @Override protected Pose2d getStartPose() { return new Pose2d(-60, 24, 0); }
    @Override protected Action buildSequence(...) { return RRPaths.bluePath(...); }
}
```

### 3. **Separate Configuration from Logic**

Keep configuration separate and easy to change:

```java
// config/RobotInitConfig.java
public class RobotInitConfig {
    public static final String LEFT_FRONT_MOTOR = "leftFront";
    public static final String RIGHT_FRONT_MOTOR = "rightFront";
    public static final double TRACK_WIDTH = 14.0;
    public static final double WHEEL_DIAMETER = 4.0;
}

// subsystems/DriveBase.java
public class DriveBase {
    public DriveBase(HardwareMap hwMap) {
        leftFront = hwMap.get(DcMotor.class, RobotInitConfig.LEFT_FRONT_MOTOR);
        // ...
    }
}
```

### 4. **Use Meaningful Comments**

Comment the "why", not the "what":

```java
// ❌ Bad - obvious
x = x + 1;  // increment x

// ✅ Good - explains reasoning
// Pinpoint requires 10ms minimum between reads to avoid stale data
Thread.sleep(10);

// ✅ Good - documents intent
// Field origin at center, +x toward audience right, +y away from driver
// Start position: right wheels on line Y (24" from center)
Pose2d startPose = new Pose2d(-60.0, +24.0 - halfTrack, 0.0);
```

### 5. **Archive Old Code, Don't Delete**

When replacing implementations:

1. Move old code to `Archive/` folder
2. Add date and reason to filename or comment
3. Keep for reference, but don't use in competition

```java
// Archive/DriveTeleOp_2024_11_15_PreSeasonVersion.java
```

### 6. **Use Consistent Formatting**

- **Indent:** 4 spaces (or tab set to 4)
- **Line length:** < 120 characters
- **Braces:** Opening brace on same line
- **Naming:** camelCase for methods/variables, PascalCase for classes

```java
public class DriveBase {
    private DcMotor leftFront;
    
    public void setDrivePower(double forward, double strafe, double turn) {
        // Implementation
    }
}
```

### 7. **Group Imports Logically**

```java
// Road Runner
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;

// FTC SDK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// Team code
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeBase;
```

### 8. **Document OpMode Purpose**

Add Javadoc to explain what each OpMode does:

```java
/**
 * BLUE alliance autonomous using Limelight vision and Road Runner path following.
 * 
 * Starting position: Square D with right wheels on line Y
 * Scoring: High chamber -> park in observation zone
 * 
 * @author Team 4BrainStorms
 * @version 2024-12-15
 */
@Autonomous(name="Auton: BLUE (LL+RR)", group="AA Auton")
public class BlueAllianceAutonRR extends LimelightAutonBaseRR {
    // Implementation
}
```

---

## 🔄 Migration Guide

### Step 1: Create New Package Structure

Create the recommended package directories:

```bash
# In TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
mkdir -p drive
mkdir -p localization
mkdir -p test
# (other directories already exist)
```

### Step 2: Move Files (Priority Order)

**High Priority - Move Immediately:**

1. **Move to `teleop/`:**
   - `DriveTeleOpSimple.java`
   - `DriveTeleOpWithMotor.java`
   - `DriveTeleOp_Tag24_Pinpoint60in.java`
   - `DriveTeleopTelemetry.java`
   - `DualControllerAlign.java`

2. **Move to `drive/`:**
   - `MecanumDrive.java`
   - `TankDrive.java`
   - `Drawing.java`

3. **Move to `localization/`:**
   - `Localizer.java`
   - `PinpointLocalizer.java`
   - `OTOSLocalizer.java`
   - `ThreeDeadWheelLocalizer.java`
   - `TwoDeadWheelLocalizer.java`
   - `GoBildaPinpointDriver.java`

4. **Move to `vision/`:**
   - `SimpleAlignToTag.java`
   - `SensorGoBildaPinpointExample.java`

5. **Move to `config/`:**
   - `TeleopTuningConfig.java`

6. **Move to `util/`:**
   - `RRPG_FieldToRR_Quick.java`

7. **Move to `test/`:**
   - From `auton/`: `PinpointSanity.java`, `RR_FollowerSanity.java`, `AxisSanity.java`, etc.

### Step 3: Update Imports

After moving files, update package declarations and imports:

```java
// Old location
package org.firstinspires.ftc.teamcode;

// New location
package org.firstinspires.ftc.teamcode.teleop;

// Update imports in other files
import org.firstinspires.ftc.teamcode.MecanumDrive;  // Old
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;  // New
```

### Step 4: Test After Each Move

After moving a group of files:

1. **Build the project:** `./gradlew build`
2. **Deploy to robot:** Run OpModes to verify they still work
3. **Check Driver Station:** Verify OpModes appear in correct groups

### Step 5: Update Documentation

Update `readme.md` to reflect the new structure:

```markdown
## Code Organization

Our code is organized into the following packages:

- `auton/` - Autonomous OpModes
- `teleop/` - TeleOp OpModes
- `drive/` - Drive system implementations
- `localization/` - Odometry and position tracking
- `subsystems/` - Robot subsystem classes
- `vision/` - Computer vision code
- `config/` - Configuration constants
- `util/` - Utility classes
- `test/` - Test and diagnostic OpModes
```

### Migration Script Example

For Android Studio / IntelliJ IDEA:

1. Right-click file → Refactor → Move
2. Select target package
3. Let IDE update imports automatically

Or use command line (requires updating imports manually):

```bash
cd TeamCode/src/main/java/org/firstinspires/ftc/teamcode

# Move teleop files
mv DriveTeleOpSimple.java teleop/
mv DriveTeleOpWithMotor.java teleop/
mv DriveTeleOp_Tag24_Pinpoint60in.java teleop/
mv DriveTeleopTelemetry.java teleop/
mv DualControllerAlign.java teleop/

# Move drive files
mv MecanumDrive.java drive/
mv TankDrive.java drive/
mv Drawing.java drive/

# Move localization files
mv Localizer.java localization/
mv PinpointLocalizer.java localization/
mv OTOSLocalizer.java localization/
mv ThreeDeadWheelLocalizer.java localization/
mv TwoDeadWheelLocalizer.java localization/
mv GoBildaPinpointDriver.java localization/
```

---

## 📊 Summary

### Quick Checklist

- [ ] Move 18 root files to appropriate subdirectories
- [ ] Update package declarations in moved files
- [ ] Update imports in files that reference moved classes
- [ ] Test build and deployment
- [ ] Verify OpModes appear on Driver Station
- [ ] Update readme.md with new structure
- [ ] Archive any old/unused code
- [ ] Document any custom decisions

### Benefits of This Organization

1. **Easier navigation** - Files are grouped by purpose
2. **Better collaboration** - Team members know where to find things
3. **Reduced conflicts** - Different developers work in different packages
4. **Faster builds** - Smaller, focused packages
5. **Clearer purpose** - Package name indicates file purpose
6. **Easier maintenance** - Related code stays together

### Questions?

If you have questions about where a specific file should go, ask yourself:

1. **Is it an OpMode?** → `auton/`, `teleop/`, `tuning/`, or `test/`
2. **Is it a drive system?** → `drive/`
3. **Is it about localization?** → `localization/`
4. **Is it a robot subsystem?** → `subsystems/`
5. **Is it vision-related?** → `vision/`
6. **Is it configuration?** → `config/`
7. **Is it a utility?** → `util/`
8. **Is it a data class?** → `messages/`

When in doubt, choose the package that best describes the **primary purpose** of the file.

---

*This guide follows FIRST Tech Challenge best practices and Road Runner conventions.*
*Updated: 2024-12-15*
