# 📋 Code Organization Migration Checklist

Use this checklist to track your progress as you reorganize the codebase.

---

## 🎯 Phase 1: Preparation

- [ ] **Backup your code**
  - [ ] Commit all current changes
  - [ ] Create a backup branch: `git branch backup-before-reorganization`
  - [ ] Push to GitHub: `git push origin backup-before-reorganization`

- [ ] **Read documentation**
  - [ ] Review `CODE_ORGANIZATION.md`
  - [ ] Review `QUICK_REFERENCE.md`
  - [ ] Review `ORGANIZATION_DIAGRAM.md`

- [ ] **Test current state**
  - [ ] Build successfully: `./gradlew build`
  - [ ] Deploy to robot
  - [ ] Test key OpModes work

---

## 🏗️ Phase 2: Create Package Structure

Create new packages (directories) for better organization:

- [ ] Create `drive/` package
  ```bash
  cd TeamCode/src/main/java/org/firstinspires/ftc/teamcode
  mkdir drive
  ```

- [ ] Create `localization/` package
  ```bash
  mkdir localization
  ```

- [ ] Create `test/` package
  ```bash
  mkdir test
  ```

---

## 📦 Phase 3: Move Files to Appropriate Packages

### 3.1: Move TeleOp OpModes to `teleop/`

- [ ] `DriveTeleOpSimple.java` → `teleop/`
  - [ ] Move file
  - [ ] Update package declaration to `package org.firstinspires.ftc.teamcode.teleop;`
  - [ ] Update imports in other files
  - [ ] Test build

- [ ] `DriveTeleOpWithMotor.java` → `teleop/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Update imports in other files
  - [ ] Test build

- [ ] `DriveTeleOp_Tag24_Pinpoint60in.java` → `teleop/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Update imports in other files
  - [ ] Test build

- [ ] `DriveTeleopTelemetry.java` → `teleop/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Update imports in other files
  - [ ] Test build

- [ ] `DualControllerAlign.java` → `teleop/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Update imports in other files
  - [ ] Test build

**Checkpoint:** Build and deploy to verify TeleOp modes still work

---

### 3.2: Move Drive System Files to `drive/`

- [ ] `MecanumDrive.java` → `drive/`
  - [ ] Move file
  - [ ] Update package declaration to `package org.firstinspires.ftc.teamcode.drive;`
  - [ ] Update imports in **all** files that use MecanumDrive
  - [ ] Test build

- [ ] `TankDrive.java` → `drive/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Update imports in other files
  - [ ] Test build

- [ ] `Drawing.java` → `drive/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Update imports in other files
  - [ ] Test build

**Checkpoint:** Build and deploy to verify drive system works

---

### 3.3: Move Localization Files to `localization/`

- [ ] `Localizer.java` → `localization/`
  - [ ] Move file
  - [ ] Update package declaration to `package org.firstinspires.ftc.teamcode.localization;`
  - [ ] Update imports in other files
  - [ ] Test build

- [ ] `PinpointLocalizer.java` → `localization/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Update imports in other files
  - [ ] Test build

- [ ] `OTOSLocalizer.java` → `localization/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Update imports in other files
  - [ ] Test build

- [ ] `ThreeDeadWheelLocalizer.java` → `localization/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Update imports in other files
  - [ ] Test build

- [ ] `TwoDeadWheelLocalizer.java` → `localization/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Update imports in other files
  - [ ] Test build

- [ ] `GoBildaPinpointDriver.java` → `localization/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Update imports in other files
  - [ ] Test build

**Checkpoint:** Build and deploy to verify localization works

---

### 3.4: Move Vision Files to `vision/`

- [ ] `SimpleAlignToTag.java` → `vision/`
  - [ ] Move file
  - [ ] Update package declaration to `package org.firstinspires.ftc.teamcode.vision;`
  - [ ] Update imports in other files
  - [ ] Test build

- [ ] `SensorGoBildaPinpointExample.java` → `vision/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Update imports in other files
  - [ ] Test build

**Checkpoint:** Build and deploy to verify vision code works

---

### 3.5: Move Configuration Files to `config/`

- [ ] `TeleopTuningConfig.java` → `config/`
  - [ ] Move file
  - [ ] Update package declaration to `package org.firstinspires.ftc.teamcode.config;`
  - [ ] Update imports in other files
  - [ ] Test build

**Checkpoint:** Build and deploy to verify configuration works

---

### 3.6: Move Utility Files to `util/`

- [ ] `RRPG_FieldToRR_Quick.java` → `util/`
  - [ ] Move file
  - [ ] Update package declaration to `package org.firstinspires.ftc.teamcode.util;`
  - [ ] Update imports in other files
  - [ ] Test build

**Checkpoint:** Build and deploy to verify utilities work

---

### 3.7: Move Test/Diagnostic OpModes from `auton/` to `test/`

- [ ] `PinpointSanity.java` → `test/`
  - [ ] Move file
  - [ ] Update package declaration to `package org.firstinspires.ftc.teamcode.test;`
  - [ ] Verify it still appears on Driver Station
  - [ ] Test build

- [ ] `RR_FollowerSanity.java` → `test/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Test build

- [ ] `AxisSanity.java` → `test/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Test build

- [ ] `PinpointHeadingCheck.java` → `test/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Test build

- [ ] `RR_AxisProbe.java` → `test/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Test build

- [ ] `RR_RawDriveProbe.java` → `test/`
  - [ ] Move file
  - [ ] Update package declaration
  - [ ] Test build

**Checkpoint:** Build and deploy to verify test OpModes work

---

## ✅ Phase 4: Final Verification

### 4.1: Build and Test

- [ ] **Clean build**
  ```bash
  ./gradlew clean
  ./gradlew build
  ```

- [ ] **No build errors**

- [ ] **Deploy to robot**

- [ ] **Test each OpMode category:**
  - [ ] Autonomous modes work
  - [ ] TeleOp modes work
  - [ ] Test/diagnostic modes work
  - [ ] Tuning modes work

### 4.2: Verify Structure

- [ ] **Root package is clean**
  - Only `readme.md` should remain in `teamcode/` root
  
- [ ] **All packages exist:**
  - [ ] `auton/`
  - [ ] `teleop/`
  - [ ] `drive/`
  - [ ] `localization/`
  - [ ] `subsystems/`
  - [ ] `vision/`
  - [ ] `config/`
  - [ ] `util/`
  - [ ] `test/`
  - [ ] `tuning/`
  - [ ] `messages/`
  - [ ] `telemetry/`
  - [ ] `debug/`
  - [ ] `Archive/`

- [ ] **Package declarations are correct** in all moved files

- [ ] **Imports are updated** in files that reference moved classes

---

## 📝 Phase 5: Documentation

- [ ] **Update `readme.md`** to reflect new structure

- [ ] **Document any custom decisions** you made

- [ ] **Add package-level documentation** (optional)
  - Create `package-info.java` files for major packages

- [ ] **Update team documentation** with new structure

---

## 🎉 Phase 6: Commit and Share

- [ ] **Stage changes**
  ```bash
  git add .
  ```

- [ ] **Commit reorganization**
  ```bash
  git commit -m "Refactor: Reorganize code into logical packages"
  ```

- [ ] **Push to GitHub**
  ```bash
  git push origin <your-branch-name>
  ```

- [ ] **Create pull request** (if using PR workflow)

- [ ] **Share with team** - Inform team members of new structure

- [ ] **Update team documentation** or wiki

---

## 🔄 Optional: Advanced Organization

Once basic organization is complete, consider these improvements:

- [ ] **Separate competition code from experiments**
  - Use groups like `"AA Auton"` for competition modes
  - Use groups like `"zz"` for experimental modes

- [ ] **Create base classes** for common patterns
  - Example: `BaseAutonomous` for all autonomous modes

- [ ] **Add package-level JavaDoc**
  - Create `package-info.java` in each package

- [ ] **Consistent naming conventions**
  - Review all file names for consistency
  - Rename files that don't follow conventions

- [ ] **Remove unused code**
  - Archive or delete truly obsolete files

---

## 📊 Progress Tracking

**Overall Progress:**
- Phase 1: Preparation - [ ] Complete
- Phase 2: Create Packages - [ ] Complete
- Phase 3: Move Files - [ ] Complete
- Phase 4: Verification - [ ] Complete
- Phase 5: Documentation - [ ] Complete
- Phase 6: Commit & Share - [ ] Complete

**Total Files to Move:** 24 files
**Files Moved:** ___ / 24
**Percentage Complete:** ___%

---

## 💡 Tips

1. **Move files in small batches** - Don't move everything at once
2. **Test after each batch** - Catch issues early
3. **Use IDE refactoring** - Let Android Studio update imports automatically
4. **Keep notes** - Document any issues you encounter
5. **Ask for help** - If stuck, consult documentation or ask team

---

## 🆘 Troubleshooting

### Build Errors After Moving Files

**Problem:** `cannot find symbol` errors
**Solution:** Update imports in affected files
```java
// Old
import org.firstinspires.ftc.teamcode.MecanumDrive;

// New
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
```

### OpModes Don't Appear on Driver Station

**Problem:** Moved OpMode doesn't show up
**Solution:** 
1. Check package declaration is correct
2. Verify `@Autonomous` or `@TeleOp` annotation is present
3. Rebuild and redeploy
4. Check Driver Station group filters

### Android Studio Shows Red Squiggles

**Problem:** IDE shows errors but code works
**Solution:**
1. File → Invalidate Caches → Invalidate and Restart
2. Build → Clean Project
3. Build → Rebuild Project

---

## 📚 Resources

- **Full Guide:** `CODE_ORGANIZATION.md`
- **Quick Reference:** `QUICK_REFERENCE.md`
- **Visual Diagram:** `ORGANIZATION_DIAGRAM.md`
- **FTC Docs:** https://ftc-docs.firstinspires.org/
- **Road Runner Docs:** https://rr.brott.dev/

---

**Good luck with your reorganization!** 🚀

*Remember: Organization is an ongoing process, not a one-time task.*
*Keep your codebase clean as you add new features.*

---

**Started:** _____________ (date)
**Completed:** _____________ (date)
**Team Members Involved:** _____________
**Notes:** _____________

