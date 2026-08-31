# Examples

Mentor sandbox for teaching examples, plus a working **Sloth** hot-reload setup.

This module is not any team's robot code. It builds its own APK, exactly like a team
module, so an example can be run on a real robot without touching a team's folder.

## The example

`ExampleMecanumTeleOp` — copied from the SDK sample `BasicOmniOpMode_Linear` and adapted.
Standard mecanum POV drive: left stick drives and strafes, right stick rotates.

It expects these four motors in the robot configuration:

```
front_left_drive    back_left_drive
front_right_drive   back_right_drive
```

Two tunables are pulled out at the top of the class, `DRIVE_SPEED` and `TURN_SPEED`,
and both are reported in telemetry. They exist so a hot reload is visible without
needing to guess whether anything changed.

## Sloth — hot reload

Normally a code change means a full reinstall: rebuild the APK, push ~50 MB, restart the
Robot Controller app. [Sloth](https://github.com/Dairy-Foundation/Sloth) sends only the
changed classes and reloads them on the robot in about a second.

Already wired up in `Examples/build.gradle`:

- `buildscript` block pulling `dev.frozenmilk:Load:0.2.4` (must be first in the file)
- `apply plugin: 'dev.frozenmilk.sinister.sloth.load'`
- `implementation 'dev.frozenmilk.sinister:Sloth:0.2.4'`
- the `https://repo.dairy.foundation/releases` maven repo

That adds four Gradle tasks: `assembleSloth`, `dexSloth`, `deploySloth`,
`removeSlothRemote`.

### Using it

1. **Install once, normally.** Connect to the robot and do a standard Run/install of the
   `Examples` module. Hot reload patches an already-installed app; it can't create one.
2. **Then edit and reload.** Change `DRIVE_SPEED` to `0.35` and run the **`deploySloth`**
   Gradle task. The new value is live in about a second.
3. **To go back** to the code that's actually installed in the APK, run
   **`removeSlothRemote`**.

In Android Studio, make a run configuration for this so it's one click:
**Run → Edit Configurations → + → Gradle**, project `Examples`, task `deploySloth`.

### What hot reload does NOT cover

> ⚠️ **Sloth only reloads classes under `org.firstinspires.ftc.teamcode`.**

This is hardcoded — `TeamCodeSearch` in Sinister 2.2.0 contains the literal string
`org.firstinspires.ftc.teamcode`. It is the reason this module's code lives in
`org.firstinspires.ftc.teamcode.examples` rather than `org.firstinspires.ftc.examples`.

**This matters if a team wants Sloth.** Every team module uses its own package —
`org.firstinspires.ftc.team22258`, `org.firstinspires.ftc.team22256`, and so on — all of
which are *outside* the tree Sloth scans. Adding the dependency to a team module would
build fine and then silently never hot reload anything. A team adopting Sloth would first
have to move their code under `org.firstinspires.ftc.teamcode.…`, which is a real
refactor and a mentor decision.

A full reinstall is also required, not a hot reload, when you:

- add or change a library in `build.gradle`
- change anything outside the scanned package
- change `@Pinned` annotations

### If it misbehaves

Run `removeSlothRemote`, then do a normal full install. That puts the robot back on
exactly the code in the APK, which rules Sloth out as a suspect.
