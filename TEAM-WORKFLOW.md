# Herobotix Repo Workflow

This is a fork of the official
[FIRST Tech Challenge FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController).
`README.md` is upstream's documentation. This file is ours.

## Who works where

| Team | Module folder | Branch |
| --- | --- | --- |
| 22256 (CIS) | `Team22256/` | `2026-cis` |
| 22257 (Spartech) | `Team22257/` | `2026-spartech` |
| 22258 (Beanie Bots) | `Team22258/` | `2026-beaniebots` |
| Class robot (Herobot) | `Team00000/` | `2026-herobot` |

**Only edit your own team's folder.** Everything else — `FtcRobotController/`,
`build.common.gradle`, `build.dependencies.gradle`, `settings.gradle`,
`gradle.properties` — belongs to the whole club. Changing those affects all
four teams, so ask a mentor first.

## The only Git you need

```
git checkout 2026-<yourteam>     # once, to get on your branch
git pull                         # start of every meeting
...write code...
git add -A && git commit -m "what you changed"
git push                         # end of every meeting
```

Two rules:

1. **Never commit to `master`.** Mentors merge branches into `master`.
2. **Never `git push --force`.** It rewrites history and breaks everyone
   else's copy. If Git says something confusing, stop and ask a mentor —
   nothing is lost, and force-pushing is how things actually do get lost.

If `master` gets a fix you need mid-season:

```
git pull origin master           # brings master's changes onto your branch
```

## Adding a library (Pedro Pathing, NextFTC, RoadRunner, FTCLib...)

Put it in **your team's** `TeamNNNNN/build.gradle`, in the `dependencies`
block at the bottom — never in the shared `build.dependencies.gradle` at the
repo root.

That root file is applied to every module, so a library added there is forced
on all four teams at one shared version. That is how one team's change stops
another team's robot from building. Per-module means Team 22256 can run Pedro
Pathing 2.0.6 while Team 22258 runs 2.0.4, and neither can break the other.
Add any custom maven `repositories { }` to your module file too.

## Old seasons

Previous seasons live inside your module at:

```
TeamNNNNN/src/main/java/org/firstinspires/ftc/teamNNNNN/legacy/<season>/
```

Every module's `build.gradle` excludes `**/legacy/**`, so this code is kept for
reference but is never compiled and never ends up on the robot. To archive a
season, copy the folders into a new `legacy/2026-2027/` — no Gradle edit needed.

Each season's final code is also tagged, e.g. `season-2025-2026-team22258`.
On GitHub, use the branch dropdown and switch to the **Tags** tab to browse any
past season exactly as it was.

## Mentor: start-of-season checklist

1. `git fetch upstream && git checkout master && git merge upstream/master`
   — merge, never rebase. Rebasing `master` forces a force-push and breaks
   every student's clone at once.
2. Build all four modules: `./gradlew clean assembleDebug`. Upstream moves
   things (in 11.1 the Android plugin moved out of `build.common.gradle` into
   each module's `build.gradle`), so expect to fix something here.
3. Tag each team's season-end commit: `season-<years>-team<number>`.
4. Merge each team branch into `master`, then archive their code into
   `legacy/<season>/`.
5. Cut new `<year>-<team>` branches from `master` and push them.
6. Rename the finished branches to `archive/<season>-<name>`.
