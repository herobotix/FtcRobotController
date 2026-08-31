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

Install **Android Studio Quail 3 (2026.1.3)** from <https://developer.android.com/studio>.

Everyone in the club runs the **same** version. Mixed versions cause confusing problems
that look like code bugs but aren't. If you already have Android Studio, check
**Android Studio → About** (macOS) or **Help → About** (Windows).

The hard floor is **2025.1.2**; anything older cannot open this project at all. We
standardize on Quail 3 because it supports Android Gradle Plugin 7.1 through 9.3, which
should carry us through the SDK update at kickoff without another Studio upgrade.

> ⚠️ Android Studio Ladybug (2024.2) does **not** work, even though the FTC SDK's own
> README says it does. That page is out of date — the SDK now ships Android Gradle
> Plugin 8.13.2, which Ladybug cannot load. You will get an error about the Android
> Gradle plugin version instead of a working project.

**Turn off automatic updates:** Settings → Appearance & Behavior → System Settings →
Updates, and uncheck automatic updates for the IDE. Studio upgrades are a mentor
decision, made between competitions — not something that should happen to your laptop
the night before one.

### b. Get the code

In Android Studio: **File → New → Project from Version Control**, and use

```
https://github.com/herobotix/FtcRobotController.git
```

Then switch to your team's branch (see the table in section 3). Do not start working
on `master`.

### c. Set the Gradle JDK to Java 17

With the project open, do this before you try to build. **It is required, and you have
to do it on your own laptop** — it is a local Android Studio setting that cannot be
stored in the repository, so nobody can do it for you.

1. Open **Settings → Build, Execution, Deployment → Build Tools → Gradle**
2. Find the **Gradle JDK** dropdown
3. Choose **Download JDK…**, then select **version 17**, vendor **Eclipse Temurin**,
   and take the newest `17.0.x` offered
4. Click **OK** and let the project re-sync

Studio picks the right build for your computer automatically — Windows laptops get the
x64 build, Apple Silicon Macs get aarch64. It is the same JDK either way.

Why 17, specifically: Android Gradle Plugin 8.13 lists JDK 17 as both its minimum and
its default, so it is the best-tested setup. It also insulates your build from Android
Studio, which bundles its own newer JDK that changes whenever Studio updates.

> **Note:** you may read online that the REV Control Hub "requires Java 17." That is not
> correct, though the advice to use 17 is still good. The Control Hub is an Android
> device and never runs a JDK at all — your code is compiled to Java 8 bytecode and then
> converted to DEX to run on Android. The Gradle JDK only runs the build on your laptop.
> See the mentor notes for the full explanation.

### d. Check it works

Let Android Studio finish "Gradle sync" — the progress bar at the bottom. The first
sync downloads a lot and can take 10+ minutes on school wifi. Then pick your team's
module from the dropdown at the top and press **Run**.

To check from a terminal instead:

```bash
./gradlew clean assembleDebug
```

`BUILD SUCCESSFUL` means your setup is good.

### Windows laptops — read this first

Most of the club is on Windows, and nearly every "it works on the mentor's laptop"
problem traces back to one of these.

**Put the project somewhere short and local.** `C:\ftc\FtcRobotController` is ideal.
Android builds create very deep folder paths, and a project buried under
`Documents\...\OneDrive\...` will hit Windows path limits and produce errors that look
like code problems.

**Do not let the project or your home folder sync to OneDrive.** School-managed laptops
often redirect `C:\Users\<you>` into OneDrive. If that happens, Gradle's cache
(`C:\Users\<you>\.gradle`) and your downloaded JDK (`C:\Users\<you>\.jdks`) get
synced too — which causes file-locking failures, mysterious build errors, and enormous
upload traffic. If you see OneDrive icons on those folders, tell a mentor before going
further.

**Antivirus makes builds slow.** Real-time scanning of the Gradle cache can turn a
20-second build into several minutes. If you have permission, exclude
`C:\Users\<you>\.gradle` from scanning. On a school-managed machine you may not, which
is worth knowing so you don't think something is broken.

**Budget disk space.** Android Studio, the Android SDK, the Gradle cache and the build
output together need roughly **15–20 GB**. A nearly-full drive fails in confusing ways.

### Versions this project uses

You don't need to set any of these — they come with the project. They're listed so a
mentor can diagnose a broken setup.

| Thing | Version | Where it's set |
| --- | --- | --- |
| Android Studio | **Quail 3, 2026.1.3** (floor: 2025.1.2) | you install it |
| Gradle JDK | **Eclipse Temurin 17** (newest 17.0.x) | Settings → Gradle → Gradle JDK, per laptop |
| Gradle | 9.1.0 | `gradle/wrapper/gradle-wrapper.properties` |
| Android Gradle Plugin | 8.13.2 | `build.gradle` |
| FTC SDK | 11.2.1 | `build.dependencies.gradle` |
| compileSdk / minSdk / targetSdk | 34 / 24 / 28 | `build.common.gradle` |
| Java language level | 8 | `build.common.gradle` |

Android Studio will offer to upgrade the **Android Gradle Plugin**, and separately the
**Gradle wrapper**. **Say no to both** and tell a mentor. Those versions come from the
FTC SDK, not from us — accepting either pushes us off what FIRST ships and breaks anyone
still on an older Studio. It's a whole-club decision.

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

### If a push gets rejected

The repository is protected, so some things simply won't go through. If you see
`GH013: Repository rule violations found`, you tried to do one of these:

| What you tried | Do this instead |
| --- | --- |
| Push to `master` | Push to your team's branch. Ask a mentor to update `master`. |
| Create a new branch | Ask a mentor. Only mentors create branches. |
| Delete a branch, or push an old branch name from last season | Nothing to do — this is expected. Clone fresh (see the note above). |
| `git push --force` | Don't. Ask a mentor — your work can almost always be saved. |

None of these mean you broke anything, and none of them lose your work. Your commits
are still on your laptop until they're pushed.

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

### Repository protection

Four rulesets are active (Settings → Rules → Rulesets). Every one bypasses on the
**Repository admin** role, so mentors are unaffected and students (write access) are
blocked.

| Ruleset | Applies to | Blocks |
| --- | --- | --- |
| Mentors only: master | `refs/heads/master` | update, delete, force-push |
| Mentors only: create, delete or force-push branches | all branches | create, delete, force-push |
| Archive branches are read-only | `refs/heads/archive/**` | update |
| Mentors only: season tags | all tags | create, update, delete |

Net effect: a student can push only to the four existing `2026-*` branches. In
particular they cannot re-create last season's deleted branch names from a stale
clone, which is the failure mode this is here to prevent.

> ⚠️ **Outstanding:** the organization's **base repository permission is `Admin`**, so
> every org member automatically gets repo admin and therefore bypasses all four
> rulesets — and can edit or delete them. Setting a lower permission on an individual
> collaborator does not help; GitHub takes the higher of base and direct grant.
>
> Fix: an **org owner** (Adam) goes to Organization Settings → Member privileges →
> Base permissions and sets it to **Read**, then grants each person their real access
> on the repo directly. Justin is an org member, not an owner, so this needs Adam or
> an ownership change. Until then the rulesets fully constrain the students (who are
> outside collaborators with write) but not org members.

### Why the Gradle JDK is pinned to 17

Students are told to set **Gradle JDK = Eclipse Temurin 17**. The reasoning is worth
recording, because the explanation circulating online is wrong.

**The claim:** "the REV Control Hub requires Java 17."

**Not true.** The Control Hub is an Android device (this project sets
`minSdkVersion 24`, Android 7.0; the Hub runs 7.1). It never runs a JDK. Source is
compiled to Java 8 bytecode by `compileOptions ... VERSION_1_8` in
`build.common.gradle`, then D8 converts it to DEX for Android's ART runtime. The Gradle
JDK is purely the JVM that runs the build on the laptop, and it does not change what
lands on the robot. Verified directly: a build run on Android Studio's bundled JBR 25
emits class files with major version 52 — Java 8 — identical to a JDK 17 build. This
project builds successfully on both 17 and 25.

**The real reasons to pin 17:**

1. AGP 8.13 lists JDK 17 as both minimum and default — the best-tested configuration.
2. FTC is pinned to Java 8 language level. JDK 25 already warns that
   `source value 8 is obsolete and will be removed in a future release`; when a JDK
   actually removes it, a newer toolchain hard-fails while 17 never will.
3. It decouples the build from Android Studio. By default the Gradle JDK resolves to
   Studio's *bundled* JBR (Quail 3 ships JBR 25), so a Studio update silently changes
   the JDK the build runs on. Pinning removes that whole class of surprise.

The setting lives in `.idea/gradle.xml` and `.gradle/config.properties`, both
gitignored — so it cannot be committed and every student must set it themselves. Check
it when helping someone debug a build.

### Known build warnings (not problems)

`./gradlew --warning-mode all` reports Groovy space-assignment and multi-string
dependency deprecations, all of which "will fail with an error in Gradle 10." We are on
Gradle 9.1, so nothing is broken. The four occurrences in files this fork owns are
fixed; the remaining three are in upstream's `build.common.gradle` (lines 49, 87, 114)
and are deliberately left alone so that file stays identical to upstream. Leave them for
FIRST to fix.

On JDK 25 you will additionally see `source/target value 8 is obsolete`. It does not
appear on JDK 17, which is one more reason for the pin above. Do not add
`android.javaCompile.suppressSourceTargetDeprecationWarning` — it is another divergence
from upstream to hide a cosmetic message.

### Gradle and AGP versions belong to FIRST

`gradle/wrapper/gradle-wrapper.properties` and the AGP version in `build.gradle` have
been set by an FTC SDK release every single time they have changed — v6.0, v7.2, v10.1,
v11.2. The club has never picked either one. When Android Studio offers to upgrade them,
decline; the next SDK sync will bring whatever FIRST chose, and a local bump only creates
a merge conflict that upstream wins anyway.

Revisit only if a concrete problem appears — for example, if Studio starts bundling a
JDK newer than the wrapper's Gradle supports. Gradle 9.1 runs on JDK 17–25; Gradle 9.4+
extends that to 26. Pinning the Gradle JDK to 17 already avoids this.

### Deliberate differences from upstream

| File | Difference | Why |
| --- | --- | --- |
| `gradle.properties` | `-Xmx4096M` instead of `-Xmx1024M` | D8 runs out of heap dexing four team modules at once. Upstream only builds one. |
| `README.md` | Ours; upstream's moved to `doc/` | Students land here first. |
| `TeamCode/` | Removed | Replaced by the four `TeamNNNNN` modules. |

Team-specific libraries live in each module's own `build.gradle`, so
`build.common.gradle` and `build.dependencies.gradle` stay byte-identical to upstream
and shouldn't conflict on a sync. Keep it that way.
