# 📋 Quick Reference: Where Does This File Go?

A simple decision tree for organizing your FTC code.

---

## 🎯 OpModes (extends LinearOpMode or OpMode)

| Type | Package | Naming | Group |
|------|---------|--------|-------|
| **Autonomous** | `auton/` | `BlueAllianceAutonRR` | `"AA Auton"` |
| **TeleOp** | `teleop/` | `DriveTeleOpSimple` | `"drive"` |
| **Tuning** | `tuning/` | `LocalizationTest` | `"tuning"` |
| **Testing** | `test/` | `PinpointSanity` | `"debug"` |

---

## 🔧 Support Classes (not OpModes)

| Purpose | Package | Example |
|---------|---------|---------|
| **Drive systems** | `drive/` | `MecanumDrive.java` |
| **Localizers** | `localization/` | `PinpointLocalizer.java` |
| **Subsystems** | `subsystems/` | `IntakeBase.java` |
| **Vision** | `vision/` | `SimpleAlignToTag.java` |
| **Configuration** | `config/` | `RobotInitConfig.java` |
| **Utilities** | `util/` | `StartPoses.java` |
| **Messages/DTOs** | `messages/` | `PoseMessage.java` |
| **Debug helpers** | `debug/` | `HeadingHoldKpStore.java` |
| **Telemetry** | `telemetry/` | `DashboardLayout.java` |

---

## 🏷️ Naming Cheat Sheet

### OpMode Names (in @Autonomous or @TeleOp annotation)

```java
// Autonomous - descriptive, includes alliance
@Autonomous(name="Auton: BLUE (LL+RR)", group="AA Auton")

// TeleOp - includes control style
@TeleOp(name="Drive TeleOp (Robot-Centric — Simple)", group="drive")

// Test/Debug - purpose clear
@Autonomous(name="Pinpoint Sanity", group="debug")
```

### Class Names (filename)

```java
// Alliance-specific: Include alliance in name
BlueAllianceAutonRR.java
RedAllianceAutonRR.java

// Generic: Descriptive of purpose
DriveTeleOpSimple.java
MecanumDrive.java
PinpointLocalizer.java

// Tests: Purpose + "Sanity", "Test", or "Check"
PinpointSanity.java
LocalizationTest.java
```

### Groups for Driver Station

| Category | Group Name | Sorts |
|----------|-----------|-------|
| Competition Auton | `"AA Auton"` | Top |
| Competition TeleOp | `"drive"` | Middle |
| Tuning | `"tuning"` | Middle |
| Testing/Debug | `"debug"` | Middle |
| Archive/Experimental | `"zz"` | Bottom |

---

## 📂 Current Files → New Location

### Move to `teleop/`
- ✅ `DriveTeleOpSimple.java`
- ✅ `DriveTeleOpWithMotor.java`
- ✅ `DriveTeleOp_Tag24_Pinpoint60in.java`
- ✅ `DriveTeleopTelemetry.java`
- ✅ `DualControllerAlign.java`

### Move to `drive/`
- ✅ `MecanumDrive.java`
- ✅ `TankDrive.java`
- ✅ `Drawing.java`

### Move to `localization/`
- ✅ `Localizer.java`
- ✅ `PinpointLocalizer.java`
- ✅ `OTOSLocalizer.java`
- ✅ `ThreeDeadWheelLocalizer.java`
- ✅ `TwoDeadWheelLocalizer.java`
- ✅ `GoBildaPinpointDriver.java`

### Move to `vision/`
- ✅ `SimpleAlignToTag.java`
- ✅ `SensorGoBildaPinpointExample.java`

### Move to `config/`
- ✅ `TeleopTuningConfig.java`

### Move to `util/`
- ✅ `RRPG_FieldToRR_Quick.java`

### Move to `test/` (from `auton/`)
- ✅ `PinpointSanity.java`
- ✅ `RR_FollowerSanity.java`
- ✅ `AxisSanity.java`
- ✅ `PinpointHeadingCheck.java`
- ✅ `RR_AxisProbe.java`
- ✅ `RR_RawDriveProbe.java`

---

## ✅ Decision Tree

```
Is it a .java file?
│
├─ YES: Is it an OpMode (has @Autonomous or @TeleOp)?
│   │
│   ├─ YES: What type?
│   │   ├─ Competition autonomous → auton/
│   │   ├─ Competition TeleOp → teleop/
│   │   ├─ Tuning/calibration → tuning/
│   │   └─ Testing/diagnostic → test/
│   │
│   └─ NO: What does it do?
│       ├─ Drives the robot → drive/
│       ├─ Tracks position → localization/
│       ├─ Robot mechanism → subsystems/
│       ├─ Computer vision → vision/
│       ├─ Configuration → config/
│       ├─ Helper functions → util/
│       ├─ Data classes → messages/
│       └─ Debug tools → debug/
│
└─ NO: Is it a README or documentation?
    └─ Keep in root or appropriate package
```

---

## 🚀 Quick Start Checklist

When adding a new file, follow these steps:

1. **Determine type** (OpMode vs. support class)
2. **Choose package** (use decision tree above)
3. **Name appropriately** (see naming cheat sheet)
4. **Set correct package declaration**
   ```java
   package org.firstinspires.ftc.teamcode.teleop;  // example
   ```
5. **Add proper annotation and group**
   ```java
   @TeleOp(name="Drive TeleOp (Simple)", group="drive")
   ```
6. **Build and test**
   ```bash
   ./gradlew build
   ```

---

## 💡 Pro Tips

1. **Use descriptive names** - Future you will thank present you
2. **Group by purpose** - Not by file type
3. **Keep OpModes simple** - Move logic to support classes
4. **Archive don't delete** - Old code is valuable reference
5. **Test after moving files** - Ensure imports are updated
6. **Document unusual choices** - Explain why in comments
7. **Be consistent** - Follow team conventions

---

## 📚 Related Files

- **Full Guide:** `CODE_ORGANIZATION.md` - Comprehensive organization guide
- **Team Docs:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md`
- **FTC Docs:** [ftc-docs.firstinspires.org](https://ftc-docs.firstinspires.org/)

---

*Print this page and keep it handy during development!*
*Team 4BrainStorms - Updated 2024-12-15*
