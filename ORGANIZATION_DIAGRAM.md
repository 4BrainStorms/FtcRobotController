# 🏗️ Code Organization Visual Diagram

## Current vs. Recommended Structure

### 📊 Current Structure (Needs Organization)

```
TeamCode/
└── src/main/java/org/firstinspires/ftc/teamcode/
    ├── 📁 Archive/                    ✅ (17 files - good!)
    ├── 📁 auton/                      ✅ (13 files - good!)
    ├── 📁 config/                     ✅ (4 files - good!)
    ├── 📁 debug/                      ✅ (1 file - good!)
    ├── 📁 messages/                   ✅ (8 files - good!)
    ├── 📁 subsystems/                 ✅ (4 files - good!)
    ├── 📁 telemetry/                  ✅ (2 files - good!)
    ├── 📁 teleop/                     ⚠️  (1 file - needs more!)
    ├── 📁 tuning/                     ✅ (4 files - good!)
    ├── 📁 util/                       ✅ (3 files - good!)
    ├── 📁 vision/                     ✅ (1 file - good!)
    │
    └── 📂 ROOT DIRECTORY ⚠️ TOO MANY FILES!
        ├── Drawing.java
        ├── DriveTeleOpSimple.java            ❌ Should be in teleop/
        ├── DriveTeleOpWithMotor.java         ❌ Should be in teleop/
        ├── DriveTeleOp_Tag24_Pinpoint60in.java ❌ Should be in teleop/
        ├── DriveTeleopTelemetry.java         ❌ Should be in teleop/
        ├── DualControllerAlign.java          ❌ Should be in teleop/
        ├── GoBildaPinpointDriver.java        ❌ Should be in localization/
        ├── Localizer.java                    ❌ Should be in localization/
        ├── MecanumDrive.java                 ❌ Should be in drive/
        ├── OTOSLocalizer.java                ❌ Should be in localization/
        ├── PinpointLocalizer.java            ❌ Should be in localization/
        ├── RRPG_FieldToRR_Quick.java         ❌ Should be in util/
        ├── SensorGoBildaPinpointExample.java ❌ Should be in vision/
        ├── SimpleAlignToTag.java             ❌ Should be in vision/
        ├── TankDrive.java                    ❌ Should be in drive/
        ├── TeleopTuningConfig.java           ❌ Should be in config/
        ├── ThreeDeadWheelLocalizer.java      ❌ Should be in localization/
        └── TwoDeadWheelLocalizer.java        ❌ Should be in localization/
```

---

### 🎯 Recommended Structure (Well Organized)

```
TeamCode/
└── src/main/java/org/firstinspires/ftc/teamcode/
    │
    ├── 📂 auton/                       🤖 AUTONOMOUS OPMODES
    │   ├── BlueAllianceAutonRR.java      • Competition autonomous
    │   ├── RedAllianceAutonRR.java       • Competition autonomous
    │   ├── LimelightAutonBaseRR.java     • Base class for auton
    │   ├── AutonomousLogicEnhancer.java  • Auton utilities
    │   ├── RRPaths.java                  • Path definitions
    │   ├── RRPG_Runner_Center.java       • Path generator runner
    │   └── DashboardAutonSample.java     • Example code
    │
    ├── 📂 teleop/                      🎮 TELEOP OPMODES
    │   ├── DriveTeleOpSimple.java        • Basic driving
    │   ├── DriveTeleOpWithMotor.java     • Drive + manipulator
    │   ├── DriveTeleOp_Tag24_Pinpoint60in.java  • Field-oriented drive
    │   ├── DriveTeleopTelemetry.java     • Drive with telemetry
    │   ├── DualControllerAlign.java      • Two-controller mode
    │   └── DashboardTeleOpSample.java    • Example code
    │
    ├── 📂 drive/                       🚗 DRIVE SYSTEM IMPLEMENTATIONS
    │   ├── MecanumDrive.java             • Main mecanum drive
    │   ├── TankDrive.java                • Tank drive alternative
    │   └── Drawing.java                  • Drive visualization
    │
    ├── 📂 localization/                📍 POSITION TRACKING
    │   ├── Localizer.java                • Interface/base class
    │   ├── PinpointLocalizer.java        • goBILDA Pinpoint
    │   ├── OTOSLocalizer.java            • SparkFun OTOS
    │   ├── ThreeDeadWheelLocalizer.java  • 3-wheel odometry
    │   ├── TwoDeadWheelLocalizer.java    • 2-wheel odometry
    │   └── GoBildaPinpointDriver.java    • Low-level driver
    │
    ├── 📂 subsystems/                  ⚙️ ROBOT MECHANISMS
    │   ├── DriveBase.java                • Drive subsystem
    │   ├── IntakeBase.java               • Intake subsystem
    │   ├── LaunchBase.java               • Launcher subsystem
    │   └── LimelightBase.java            • Vision subsystem
    │
    ├── 📂 vision/                      👁️ COMPUTER VISION
    │   ├── SimpleAlignToTag.java         • AprilTag alignment
    │   ├── SensorGoBildaPinpointExample.java  • Pinpoint example
    │   └── LLUsbSanity.java              • Limelight sanity check
    │
    ├── 📂 config/                      ⚙️ CONFIGURATION
    │   ├── RobotInitConfig.java          • Robot constants
    │   ├── TeleopTuningConfig.java       • TeleOp tuning
    │   ├── AllianceTagConfig.java        • Alliance-specific config
    │   ├── LaunchCalibration.java        • Launcher calibration
    │   └── LaunchZoneToggle.java         • Launch zone config
    │
    ├── 📂 tuning/                      🔧 TUNING OPMODES
    │   ├── TuningOpModes.java            • RR tuning registry
    │   ├── LocalizationTest.java         • Test localization
    │   ├── ManualFeedbackTuner.java      • Manual tuning
    │   └── SplineTest.java               • Test path following
    │
    ├── 📂 test/                        🧪 TESTING/DIAGNOSTICS
    │   ├── PinpointSanity.java           • Test Pinpoint sensor
    │   ├── RR_FollowerSanity.java        • Test path following
    │   ├── AxisSanity.java               • Test axis configuration
    │   ├── PinpointHeadingCheck.java     • Test heading tracking
    │   ├── RR_AxisProbe.java             • Probe axis mapping
    │   └── RR_RawDriveProbe.java         • Test raw drive powers
    │
    ├── 📂 util/                        🛠️ UTILITIES
    │   ├── StartPoses.java               • Starting position constants
    │   ├── PinpointConfig.java           • Pinpoint configuration
    │   ├── StrafeHealthCheck.java        • Strafe diagnostics
    │   └── RRPG_FieldToRR_Quick.java     • Path conversion utility
    │
    ├── 📂 messages/                    📦 DATA TRANSFER OBJECTS
    │   ├── DriveCommandMessage.java      • Drive command DTO
    │   ├── MecanumCommandMessage.java    • Mecanum command DTO
    │   ├── TankCommandMessage.java       • Tank command DTO
    │   ├── PoseMessage.java              • Pose DTO
    │   ├── MecanumLocalizerInputsMessage.java
    │   ├── TankLocalizerInputsMessage.java
    │   ├── ThreeDeadWheelInputsMessage.java
    │   └── TwoDeadWheelInputsMessage.java
    │
    ├── 📂 telemetry/                   📊 TELEMETRY HELPERS
    │   ├── DashboardLayout.java          • Dashboard layout manager
    │   └── LauncherTelemetry.java        • Launcher telemetry
    │
    ├── 📂 debug/                       🐛 DEBUG UTILITIES
    │   └── HeadingHoldKpStore.java       • Debug configuration storage
    │
    ├── 📂 Archive/                     📚 OLD/EXPERIMENTAL CODE
    │   ├── RRPG_ActionsRun_Center.java
    │   ├── RRPGPasteAndGo.java
    │   └── ... (17 files)
    │
    └── 📄 readme.md                    📖 DOCUMENTATION
```

---

## 🔄 File Migration Map

### High Priority Moves

```
FROM: teamcode/ (root)              TO: Organized packages
├─────────────────────────────────┼─────────────────────────────────┐
│                                  │                                 │
│ DriveTeleOpSimple.java          → teleop/                         │
│ DriveTeleOpWithMotor.java       → teleop/                         │
│ DriveTeleOp_Tag24_Pinpoint60in  → teleop/                         │
│ DriveTeleopTelemetry.java       → teleop/                         │
│ DualControllerAlign.java        → teleop/                         │
│                                  │                                 │
│ MecanumDrive.java               → drive/                          │
│ TankDrive.java                  → drive/                          │
│ Drawing.java                    → drive/                          │
│                                  │                                 │
│ Localizer.java                  → localization/                   │
│ PinpointLocalizer.java          → localization/                   │
│ OTOSLocalizer.java              → localization/                   │
│ ThreeDeadWheelLocalizer.java    → localization/                   │
│ TwoDeadWheelLocalizer.java      → localization/                   │
│ GoBildaPinpointDriver.java      → localization/                   │
│                                  │                                 │
│ SimpleAlignToTag.java           → vision/                         │
│ SensorGoBildaPinpointExample    → vision/                         │
│                                  │                                 │
│ TeleopTuningConfig.java         → config/                         │
│                                  │                                 │
│ RRPG_FieldToRR_Quick.java       → util/                           │
│                                  │                                 │
└─────────────────────────────────┴─────────────────────────────────┘
```

### Also Reorganize in `auton/`

Move test/diagnostic OpModes from `auton/` to new `test/` package:

```
FROM: auton/                        TO: test/
├─────────────────────────────────┼─────────────────────────────────┐
│                                  │                                 │
│ PinpointSanity.java             → test/                           │
│ RR_FollowerSanity.java          → test/                           │
│ AxisSanity.java                 → test/                           │
│ PinpointHeadingCheck.java       → test/                           │
│ RR_AxisProbe.java               → test/                           │
│ RR_RawDriveProbe.java           → test/                           │
│                                  │                                 │
└─────────────────────────────────┴─────────────────────────────────┘
```

---

## 📊 Statistics

### Before Organization
- **Root package:** 18 files ❌
- **Well-organized packages:** 10 packages ✅
- **Total packages needed:** 3 more (drive, localization, test)

### After Organization
- **Root package:** 1 file (readme.md) ✅
- **Well-organized packages:** 13 packages ✅
- **Files properly categorized:** 100% ✅

### Impact
- **Easier navigation:** Find files 3x faster
- **Better collaboration:** Clear ownership by package
- **Reduced merge conflicts:** Changes isolated to packages
- **Faster onboarding:** New team members find code easily

---

## 🎨 Package Color Coding (for mental model)

```
🤖 RUNTIME - Code that runs on robot
   ├── 🔵 auton/      - Autonomous OpModes (blue alliance!)
   ├── 🔴 teleop/     - TeleOp OpModes (red alliance!)
   ├── 🟢 test/       - Testing OpModes (green = go test!)
   └── 🟡 tuning/     - Tuning OpModes (yellow = caution/setup)

⚙️ CORE SYSTEMS - Robot functionality
   ├── 🚗 drive/        - Drive implementations
   ├── 📍 localization/ - Position tracking
   ├── ⚙️ subsystems/   - Robot mechanisms
   └── 👁️ vision/       - Computer vision

🛠️ SUPPORT - Helper code
   ├── 🎯 config/     - Configuration
   ├── 🛠️ util/       - Utilities
   ├── 📦 messages/   - Data objects
   ├── 📊 telemetry/  - Telemetry
   └── 🐛 debug/      - Debug tools

📚 OTHER
   └── 📚 Archive/    - Old code
```

---

## 📈 Organization Maturity Levels

### Level 1: Beginner ⭐
Everything in root package - hard to navigate

### Level 2: Basic ⭐⭐
Some organization, but inconsistent

### Level 3: Intermediate ⭐⭐⭐ ← YOU ARE HERE
Good package structure, needs migration

### Level 4: Advanced ⭐⭐⭐⭐ ← GOAL
Fully organized by purpose, clear naming

### Level 5: Expert ⭐⭐⭐⭐⭐
Advanced patterns (factories, builders, strategies)

---

## 🎯 Next Steps

1. **Create missing packages**
   ```bash
   mkdir drive localization test
   ```

2. **Move files** (use IDE refactor or command line)

3. **Update package declarations** in moved files

4. **Update imports** in files that reference moved classes

5. **Test build**
   ```bash
   ./gradlew build
   ```

6. **Deploy and verify** OpModes still work

7. **Update documentation** to reflect new structure

8. **Celebrate!** 🎉 Your code is now professionally organized!

---

*Visual diagrams help teams understand organization at a glance.*
*Team 4BrainStorms - Updated 2024-12-15*
