# Robot configurations

Hardware configurations for the dev board, kept here so they can be version controlled
and reinstalled without rebuilding one by hand.

## DevBoard.xml

The 3D-printed dev/test board. **Control Hub only — no Expansion Hub.**

| Device | Port | Config name | Part |
| --- | --- | --- | --- |
| Motor | 0 | `test_motor` | REV UltraPlanetary HD Hex Motor |
| Servo | 0 | `test_servo` | REV Smart Robot Servo (REV-41-1097) |
| Touch sensor | Digital **1** | `test_touch` | REV Touch Sensor |
| Distance sensor | I2C bus 0 | `test_distance` | REV 2m Distance Sensor (VL53L0X, addr 0x29) |
| IMU | I2C 0 | `imu` | Control Hub internal BHI260AP |

Used by the `Dev Board: Motor + Servo Test` and `Dev Board: Sensor Control Test` OpModes.

> **The touch sensor is on digital channel 1, not 0.** Plugged into the "0,1" digital
> port, a REV Touch Sensor is wired to pin **1** — the 4-wire JST cable leaves pin 0
> disconnected. Configure it on 0 and it silently never reads, with no error to explain
> why. This catches people every season.

> The distance sensor shares I2C bus 0 with the Control Hub's internal IMU. That is fine
> — they have different addresses (0x29 vs the IMU's) — and both were confirmed loading
> together on 2026-09-08.

> The Smart Robot Servo ships as a standard **270° positional** servo, which is why it is
> configured as `<Servo>` rather than `<ContinuousRotationServo>`. It *can* be switched to
> continuous rotation, but only with REV's separate SRS Programmer. If someone has done
> that to a servo, this config is wrong for it and the code needs `CRServo` instead.

## Installing a configuration without a Driver Hub

A hardware configuration is only two things: an XML file in `/sdcard/FIRST/` on the hub,
and a preference naming the active one. The Driver Hub is the usual *editor*, but nothing
requires it. With the hub on USB:

```bash
# 1. copy the configuration onto the hub
adb push Examples/config/DevBoard.xml /sdcard/FIRST/DevBoard.xml

# 2. stop the app so it cannot overwrite its preferences on exit
adb shell am force-stop com.qualcomm.ftcrobotcontroller

# 3. pull the preferences out, edit, and put them back
adb shell "run-as com.qualcomm.ftcrobotcontroller cat \
  shared_prefs/com.qualcomm.ftcrobotcontroller_preferences.xml" > prefs.xml
```

In `prefs.xml`, set `pref_hardware_config_filename` to name the configuration — note the
value is JSON with the quotes XML-escaped:

```
{&quot;isDirty&quot;:false,&quot;location&quot;:&quot;LOCAL_STORAGE&quot;,&quot;name&quot;:&quot;DevBoard&quot;,&quot;resourceId&quot;:0}
```

```bash
# 4. push it back and restart the app
adb push prefs.xml /data/local/tmp/prefs.xml
adb shell "run-as com.qualcomm.ftcrobotcontroller cp /data/local/tmp/prefs.xml \
  shared_prefs/com.qualcomm.ftcrobotcontroller_preferences.xml"
adb shell am start -n com.qualcomm.ftcrobotcontroller/\
org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity
```

Confirm it took by forwarding the Robot Controller console and reading **Manage →
Active Configuration**:

```bash
adb forward tcp:8080 tcp:8080
```

`run-as` works here only because the Robot Controller is a **debug** build. It will not
work against a release build.

## Why the stock configuration does not fit the dev board

`Starter bot DECODE`, which ships on the hub, expects an Expansion Hub at address 2 and
puts its servo there. On a Control-Hub-only board those devices cannot be found, so the
configuration fails to apply cleanly. `DevBoard.xml` declares only what is physically
present.
