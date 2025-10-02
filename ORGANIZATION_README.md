# 📚 Code Organization Documentation

Welcome to the FTC Robot Controller code organization guide! This collection of documents will help you organize your team's code professionally.

---

## 📖 Documentation Files

This guide consists of four main documents:

### 1. 📘 [CODE_ORGANIZATION.md](./CODE_ORGANIZATION.md) - **Comprehensive Guide**
The complete reference for organizing your FTC codebase.

**What's inside:**
- Current structure analysis
- Recommended package structure
- File organization guidelines
- Naming conventions
- Best practices for FTC teams
- Detailed migration guide with examples

**When to use:** 
- Setting up a new project
- Major code reorganization
- Creating team coding standards
- Onboarding new developers

**Length:** ~400 lines | **Read time:** 15-20 minutes

---

### 2. 📙 [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) - **Cheat Sheet**
A one-page reference for quick lookups during development.

**What's inside:**
- Decision tree for file placement
- Package naming lookup table
- OpMode naming templates
- Current files → new location mapping

**When to use:**
- Adding new files to the project
- Quick lookup during development
- Team meetings and discussions
- Print and keep at workstation

**Length:** ~150 lines | **Read time:** 5 minutes | **Printable:** ✅

---

### 3. 📗 [ORGANIZATION_DIAGRAM.md](./ORGANIZATION_DIAGRAM.md) - **Visual Guide**
Visual diagrams showing structure before and after organization.

**What's inside:**
- Current vs. recommended structure diagrams
- File migration map
- Statistics and impact analysis
- Color-coded package guide
- Organization maturity levels

**When to use:**
- Understanding the big picture
- Team presentations
- Explaining organization to new members
- Visual learners

**Length:** ~200 lines | **Read time:** 10 minutes | **Visual:** ✅

---

### 4. 📕 [MIGRATION_CHECKLIST.md](./MIGRATION_CHECKLIST.md) - **Action Plan**
Step-by-step checklist for executing the reorganization.

**What's inside:**
- Phase-by-phase migration plan
- Detailed checklist for each file
- Build and test checkpoints
- Troubleshooting guide
- Progress tracking

**When to use:**
- Actively reorganizing code
- Tracking migration progress
- Ensuring no steps are missed
- Team coordination

**Length:** ~300 lines | **Read time:** Continuous during migration | **Actionable:** ✅

---

## 🎯 How to Use This Guide

### For First-Time Setup

1. **Read** [CODE_ORGANIZATION.md](./CODE_ORGANIZATION.md) to understand the complete approach
2. **Review** [ORGANIZATION_DIAGRAM.md](./ORGANIZATION_DIAGRAM.md) to visualize the target structure
3. **Follow** [MIGRATION_CHECKLIST.md](./MIGRATION_CHECKLIST.md) to execute the reorganization
4. **Keep** [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) handy for daily use

### For Daily Development

1. **Use** [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) when adding new files
2. **Refer to** [CODE_ORGANIZATION.md](./CODE_ORGANIZATION.md) for naming conventions
3. **Share** these docs with new team members

### For Team Meetings

1. **Present** [ORGANIZATION_DIAGRAM.md](./ORGANIZATION_DIAGRAM.md) to show current state
2. **Discuss** [MIGRATION_CHECKLIST.md](./MIGRATION_CHECKLIST.md) to plan work
3. **Assign** sections of the checklist to team members

---

## 🚀 Quick Start (5 Minutes)

**Just want to get started?** Here's the express version:

1. **Create missing packages:**
   ```bash
   cd TeamCode/src/main/java/org/firstinspires/ftc/teamcode
   mkdir drive localization test
   ```

2. **Move TeleOp files:**
   ```bash
   mv DriveTeleOp*.java teleop/
   mv DualControllerAlign.java teleop/
   ```

3. **Move Drive files:**
   ```bash
   mv MecanumDrive.java drive/
   mv TankDrive.java drive/
   mv Drawing.java drive/
   ```

4. **Move Localization files:**
   ```bash
   mv *Localizer.java localization/
   mv GoBildaPinpointDriver.java localization/
   ```

5. **Update package declarations** in moved files:
   ```java
   package org.firstinspires.ftc.teamcode.teleop;  // for TeleOp files
   package org.firstinspires.ftc.teamcode.drive;   // for Drive files
   package org.firstinspires.ftc.teamcode.localization;  // for Localizer files
   ```

6. **Build and test:**
   ```bash
   ./gradlew build
   ```

**Note:** This is the minimal quick start. For a complete, safe migration, follow the full [MIGRATION_CHECKLIST.md](./MIGRATION_CHECKLIST.md).

---

## 📋 Summary of Recommendations

### Current State
- ✅ **Good:** 10 organized packages with 41 files
- ⚠️ **Needs work:** 18 files in root package
- ✅ **Good:** Archive folder with old code

### Target State
- ✅ All files in logical packages
- ✅ Root package clean (only readme.md)
- ✅ 13 well-organized packages
- ✅ Easy to find and understand code

### Key Packages to Add
1. `drive/` - Drive system implementations
2. `localization/` - Position tracking
3. `test/` - Testing and diagnostics

### Files to Reorganize
**High priority (must move):**
- 5 TeleOp files → `teleop/`
- 3 Drive files → `drive/`
- 6 Localization files → `localization/`
- 2 Vision files → `vision/`
- 2 Config files → `config/` & `util/`
- 6 Test OpModes from `auton/` → `test/`

---

## 🎓 Key Principles

### 1. **Organize by Purpose, Not Type**
Group files by what they do (e.g., `teleop/`) not by what they are (e.g., `opmodes/`)

### 2. **One Public Class Per File**
Each `.java` file should have exactly one public class matching the filename

### 3. **Clear, Descriptive Names**
Use names that explain what the code does: `BlueAllianceAutonRR` not `Auto1`

### 4. **Consistent Package Structure**
All team members should follow the same organization pattern

### 5. **Archive, Don't Delete**
Keep old code in `Archive/` for reference

---

## 📊 Impact of Good Organization

### Before Organization
- ❌ Hard to find files
- ❌ Confusion about where new code goes
- ❌ Merge conflicts from everyone editing same package
- ❌ Slow onboarding for new members

### After Organization
- ✅ Files easy to find by purpose
- ✅ Clear conventions for new code
- ✅ Reduced merge conflicts
- ✅ Faster onboarding (10 min vs 1 hour)

---

## 🛠️ Tools and IDE Support

### Android Studio / IntelliJ IDEA
- **Refactor → Move:** Automatically updates imports
- **Find Usages:** See where classes are used
- **Refactor → Rename:** Safe renaming with import updates

### Command Line
- **mkdir:** Create new package directories
- **mv:** Move files manually (requires manual import updates)
- **git mv:** Move with git tracking

### Gradle
- **./gradlew build:** Build and check for errors
- **./gradlew clean:** Clean build artifacts

---

## 🤝 Team Workflow

### For Team Lead
1. Review documentation
2. Plan reorganization timeline
3. Assign migration tasks
4. Review and merge changes

### For Developers
1. Understand new structure
2. Follow naming conventions
3. Use Quick Reference for new files
4. Keep code organized as you develop

### For New Members
1. Read CODE_ORGANIZATION.md
2. Study existing organized code
3. Use QUICK_REFERENCE.md
4. Ask questions when unsure

---

## ❓ FAQ

### Q: Do we have to reorganize everything at once?
**A:** No! You can reorganize incrementally. Start with TeleOp files, then Drive, then Localization, etc.

### Q: Will this break our existing code?
**A:** No, if you update package declarations and imports correctly. The migration checklist includes test checkpoints.

### Q: What if we use a different structure?
**A:** These are recommendations, not requirements. Adapt to your team's needs, but stay consistent.

### Q: How do we handle merge conflicts during reorganization?
**A:** Do the reorganization in a separate branch, then merge when complete. Avoid developing new features during reorganization.

### Q: Should we reorganize Archive files too?
**A:** No, leave Archive files as-is. They're historical reference and don't need to be organized.

---

## 📞 Getting Help

### If you need help:

1. **Check troubleshooting section** in [MIGRATION_CHECKLIST.md](./MIGRATION_CHECKLIST.md)
2. **Review examples** in [CODE_ORGANIZATION.md](./CODE_ORGANIZATION.md)
3. **Ask your team** - Someone may have encountered the issue
4. **Check FTC forums** - Community support at ftc-community.org
5. **Review FTC docs** - Official documentation at ftc-docs.firstinspires.org

---

## 📈 Next Steps

1. **Read** the comprehensive guide (CODE_ORGANIZATION.md)
2. **Plan** your reorganization timeline
3. **Follow** the migration checklist
4. **Test** thoroughly after each phase
5. **Document** your team's specific conventions
6. **Share** with your team

---

## 🎉 Benefits You'll See

After completing this organization:

- ✅ **Faster development** - Find files 3x quicker
- ✅ **Better collaboration** - Clear code ownership
- ✅ **Easier onboarding** - New members productive in hours, not days
- ✅ **Fewer bugs** - Related code stays together
- ✅ **Professional codebase** - Impress judges and mentors
- ✅ **Easier maintenance** - Updates isolated to specific packages

---

## 📅 Maintenance

Code organization is ongoing:

- **Weekly:** Ensure new files go in correct packages
- **Monthly:** Review for files that could be better organized
- **Seasonally:** Major reorganization if structure isn't working
- **Yearly:** Archive old code from previous season

---

## 🏆 Success Criteria

You'll know your organization is successful when:

- ✅ New team members can find any file in < 1 minute
- ✅ Everyone agrees on where new code should go
- ✅ Root package is clean (only readme)
- ✅ Build times are fast
- ✅ Merge conflicts are rare
- ✅ Code reviews focus on logic, not structure

---

## 📝 Document Versions

| Document | Version | Last Updated |
|----------|---------|--------------|
| CODE_ORGANIZATION.md | 1.0 | 2024-12-15 |
| QUICK_REFERENCE.md | 1.0 | 2024-12-15 |
| ORGANIZATION_DIAGRAM.md | 1.0 | 2024-12-15 |
| MIGRATION_CHECKLIST.md | 1.0 | 2024-12-15 |
| ORGANIZATION_README.md | 1.0 | 2024-12-15 |

---

## 📄 License

These documentation files are provided as a guide for FTC teams. Feel free to adapt them to your team's specific needs.

---

**Ready to get started?** Pick the document that matches your needs and dive in!

**Need the big picture?** → [CODE_ORGANIZATION.md](./CODE_ORGANIZATION.md)  
**Want quick answers?** → [QUICK_REFERENCE.md](./QUICK_REFERENCE.md)  
**Prefer visual guides?** → [ORGANIZATION_DIAGRAM.md](./ORGANIZATION_DIAGRAM.md)  
**Ready to reorganize?** → [MIGRATION_CHECKLIST.md](./MIGRATION_CHECKLIST.md)  

---

*Good luck with your code organization!* 🚀

*Team 4BrainStorms - Documentation created with ❤️ for FTC teams*
