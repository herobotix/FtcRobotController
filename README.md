# Herobotix — FTC Robot Controller

This is the Herobotix club's fork of the official
[FIRST Tech Challenge FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController).
It holds the robot code for our three competition teams and the school class robot.

The FTC SDK's own documentation — release notes, sample descriptions, wiring guides —
is unchanged and lives at **[doc/FTC-SDK-README.md](doc/FTC-SDK-README.md)**.

---

## 1. Set up your laptop

Do these four things in order. Ask a mentor if any step fails — don't guess.

### a. Install Android Studio

**You need Android Studio Narwhal Feature Drop (2025.1.2) or newer.** Older versions
cannot open this project at all. If you already have Android Studio, check
**Android Studio → About** (macOS) or **Help → About** (Windows) and upgrade if the
version number is lower than 2025.1.2.

> ⚠️ Android Studio Ladybug (2024.2) does **not** work, even though the FTC SDK's own
> README says it does. That page is out of date — the SDK now ships Android Gradle
> Plugin 8.13.2, which Ladybug cannot load. You will get an error about the Android
> Gradle plugin version instead of a working project.

Download from <https://developer.android.com/studio>. Take the current stable release.

Everyone on the club should run the **same** version. Mixed versions cause confusing
problems that look like code bugs but aren't. Check with a mentor before upgrading
mid-season.

### b. Java

You do **not** need to install Java separately. Android Studio bundles its own JDK and
uses it automatically. If you build from a terminal instead, you need **JDK 17**.

### c. Get the code

In Android Studio: **File → New → Project from Version Control**, and use

```
https://github.com/herobotix/FtcRobotController.git
```

Then switch to your team's branch (see the table in section 3). Do not start working
on `master`.

### d. Check it works

Let Android Studio finish "Gradle sync" — the progress bar at the bottom. The first
sync downloads a lot and can take 10+ minutes on school wifi. Then pick your team's
module from the dropdown at the top and press **Run**.

To check from a terminal instead:

```bash
./gradlew clean assembleDebug
```

`BUILD SUCCESSFUL` means your setup is good.

### Versions this project uses

You don't need to set any of these — they come with the project. They're listed so a
mentor can diagnose a broken setup.

| Thing | Version | Where it's set |
| --- | --- | --- |
| Android Studio | **2025.1.2 or newer** (current stable recommended) | you install it |
| JDK | **17** (Android Studio's bundled JDK is fine) | your machine |
| Gradle | 9.1.0 | `gradle/wrapper/gradle-wrapper.properties` |
| Android Gradle Plugin | 8.13.2 | `build.gradle` |
| FTC SDK | 11.2.1 | `build.dependencies.gradle` |
| compileSdk / minSdk / targetSdk | 34 / 24 / 28 | `build.common.gradle` |
| Java language level | 8 | `build.common.gradle` |

If Android Studio offers to "upgrade the Android Gradle Plugin," **say no** and tell a
mentor. That's a whole-club decision.

---

## 2. Folder structure — what to touch

| Folder | What it is | Can you edit it? |
| --- | --- | --- |
| `Team22256/` | Team 22256 (CIS) robot code | ✅ if you're on 22256 |
| `Team22257/` | Team 22257 (Spartech) robot code | ✅ if you're on 22257 |
| `Team22258/` | Team 22258 (Beanie Bots) robot code | ✅ if you're on 22258 |
| `Team00000/` | Class robot (Herobot) | ✅ if you're in the class |
| `FtcRobotController/` | The FTC SDK itself, plus all the sample opmodes | ❌ never |
| `build.common.gradle` | Build settings shared by all four teams | ❌ mentors only |
| `build.dependencies.gradle` | FTC SDK library versions, shared by all four teams | ❌ mentors only |
| `settings.gradle` | Which modules exist | ❌ mentors only |
| `gradle.properties`, `gradle/`, `gradlew` | Gradle itself | ❌ mentors only |
| `libs/` | Signing keystore | ❌ mentors only |
| `doc/` | FTC's documentation | ❌ mentors only |

**The rule: stay inside your own `TeamNNNNN/` folder.** Everything else is shared by
all four teams, so a change there can stop the other three teams from building on the
day before a competition. If you think something shared needs to change, ask a mentor —
sometimes it does, and that's fine, it just isn't a solo decision.

Your code goes in:

```
TeamNNNNN/src/main/java/org/firstinspires/ftc/teamNNNNN/
```

### Adding a library (Pedro Pathing, NextFTC, RoadRunner, FTCLib…)

Put it in **your team's** `TeamNNNNN/build.gradle`, in the `dependencies` block at the
bottom. Add any custom `repositories { }` there too.

**Never** add a library to `build.dependencies.gradle` at the repo root. That file is
applied to every module, so a library added there is forced on all four teams at one
shared version — that's exactly how one team's change breaks another team's robot.
Per-module means Team 22256 can run Pedro Pathing 2.0.6 while Team 22258 runs 2.0.4,
and neither can break the other. `Team22256/build.gradle` is a good example to copy.

### Old seasons

Previous seasons are kept inside each module at:

```
TeamNNNNN/src/main/java/org/firstinspires/ftc/teamNNNNN/legacy/<season>/
```

Every module's `build.gradle` excludes `**/legacy/**`, so this code is there to read and
copy from, but is never compiled and never reaches the robot. Errors shown in those
files are expected and harmless.

Each season's final code is also **tagged**. On GitHub, use the branch dropdown and
switch to the **Tags** tab to browse a past season exactly as it was:

| Tag | What |
| --- | --- |
| `season-2025-2026-team22256` | Team 22256, end of DECODE |
| `season-2025-2026-team22257` | Team 22257, end of DECODE |
| `season-2025-2026-team22258` | Team 22258, end of DECODE |
| `season-2024-2025-team00000` | Class robot, end of INTO THE DEEP |

---

## 3. Branches

Each team works on its own branch, so one team's mistake can't stop another team from
building.

| Branch | Who works on it |
| --- | --- |
| `2026-cis` | Team 22256 |
| `2026-spartech` | Team 22257 |
| `2026-beaniebots` | Team 22258 |
| `2026-herobot` | The class robot |
| `master` | **Mentors only.** The known-good baseline all team branches start from. |
| `archive/*` | Finished seasons, kept read-only for reference. |

### The Git you actually need

```bash
git checkout 2026-<yourteam>     # once, to get on your branch
git pull                         # start of every meeting
# ...write code...
git add -A
git commit -m "what you changed"
git push                         # end of every meeting
```

Two rules:

1. **Never commit to `master`.** Mentors merge team branches into `master` at the end
   of the season.
2. **Never use `git push --force`.** It rewrites history and breaks everyone else's
   copy of the project. If Git says something confusing, stop and ask a mentor —
   nothing is lost yet, and force-pushing is how things actually do get lost.

If `master` gets a fix you need mid-season:

```bash
git pull origin master           # brings master's changes onto your branch
```

> **Returning from last season?** The old branch names (`beaniebots-main`, `cis-main`,
> `spartech-main`, `herobot-main`) are gone — renamed to `archive/…`. The simplest fix
> is to delete your old project folder and clone fresh, then check out this season's
> branch. Nothing is lost; every old branch and commit is still on GitHub under
> `archive/`.

---

## 4. Mentor notes

### Start-of-season checklist

1. **Sync the SDK — merge, never rebase.**
   ```bash
   git fetch upstream
   git checkout master
   git merge upstream/master
   ```
   Rebasing `master` would force a force-push and break every student's clone at once.
   Expect a conflict in `README.md` (ours vs upstream's) — keep ours, and refresh
   `doc/FTC-SDK-README.md` from upstream's version. `gradle.properties` also differs by
   one line (heap size, see below).

2. **Build all four modules before telling anyone it's ready:**
   ```bash
   ./gradlew clean assembleDebug
   ```
   Upstream moves things between releases. In 11.1 the Android plugin moved out of
   `build.common.gradle` into each module's `build.gradle`; AGP 8.13 stopped accepting
   `package=` in `AndroidManifest.xml`. Both broke the build silently until compiled.

3. **Check the required Android Studio version** for the new AGP in `build.gradle`
   against <https://developer.android.com/build/releases/about-agp>, and tell the
   students before the first meeting. Upstream's README is not reliable on this.

4. Tag each team's season-end commit: `season-<years>-team<number>`.

5. Merge each team branch into `master`, then archive their code into `legacy/<season>/`.

6. Cut new `<year>-<team>` branches from `master` and push them.

7. Rename finished branches to `archive/<season>-<name>`.

### Deliberate differences from upstream

| File | Difference | Why |
| --- | --- | --- |
| `gradle.properties` | `-Xmx4096M` instead of `-Xmx1024M` | D8 runs out of heap dexing four team modules at once. Upstream only builds one. |
| `README.md` | Ours; upstream's moved to `doc/` | Students land here first. |
| `TeamCode/` | Removed | Replaced by the four `TeamNNNNN` modules. |

Team-specific libraries live in each module's own `build.gradle`, so
`build.common.gradle` and `build.dependencies.gradle` stay byte-identical to upstream
and shouldn't conflict on a sync. Keep it that way.
