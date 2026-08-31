# Team00000 (class robot) archive

Nothing in this folder is compiled — `Team00000/build.gradle` excludes
`**/legacy/**`. It is here to read, copy from, and learn from.

## `2024-2025/`

The 2024-2025 class code exactly as it stood on `master`, and still the code
living in `Team00000/`'s active folders. RoadRunner **0.5**
(`com.acmerobotics.roadrunner:core:0.5.6`), which is the version the active
module's dependencies are pinned to.

## `2024-2025-herobot-main/`

The same season from the `herobot-main` branch, which was never merged into
`master` (last commit Feb 2025, `0de95e9`, tagged `season-2024-2025-team00000`).
It is a reorganized and further-developed version of the above:

| Folder | What it is |
| --- | --- |
| `NEW_RR/`, `roadrunner/` | A port to RoadRunner **1.0** — the current, supported version. Not present anywhere else in this repo. |
| `OLD_RR/`, `OLD Modes/` | The RoadRunner 0.5 code, rearranged. Same work as `2024-2025/`. |
| `autonomous/`, `teleop/` | `AutoOpMain`, `AutoOpTest`, `TeleOpMain` — newer opmodes not on `master`. |

This branch was left unmerged deliberately during the 2026 season setup: the
RoadRunner 1.0 code needs dependencies the module does not currently declare,
and `OLD Modes` cannot be a Java package name because of the space.

**If the class wants to pick up the RoadRunner 1.0 work**, the path is: copy
`NEW_RR/` (or `roadrunner/`) into the module's active source folder, and add
RoadRunner 1.0 to `Team00000/build.gradle` — not to the shared
`build.dependencies.gradle`. Team 22256 already declares a working set to copy:

    implementation 'com.acmerobotics.roadrunner:ftc:0.1.19'
    implementation 'com.acmerobotics.roadrunner:core:1.0.1'
    implementation 'com.acmerobotics.roadrunner:actions:1.0.1'

Ask a mentor before starting — it replaces how the whole drivetrain is driven.
