package org.firstinspires.ftc.teamcode.examples;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Smoke test for the Sloth + FTControl Panels setup. Proves the whole loop works
 * without needing a robot: no Driver Hub, no motors, no servos, nothing plugged in.
 *
 * IMPORTANT: this OpMode touches NO hardware. It never calls hardwareMap. That is
 * deliberate -- it runs on a bare Control Hub sitting on a desk, so you can verify a
 * setup before the robot is built, or when you're away from it.
 *
 * What it demonstrates:
 *
 *   1. Running an OpMode with no Driver Station, by pressing Initialize and Start in
 *      the Panels web UI.
 *   2. Telemetry reaching both the normal Driver Station AND the Panels Telemetry
 *      panel, via JoinedTelemetry.
 *   3. Live tuning -- MESSAGE and COUNT_BY are @Configurable, so they can be edited
 *      in the Panels Configurables panel while the OpMode runs.
 *   4. Hot reload -- edit RELOAD_TAG below, run the deploySloth Gradle task, and the
 *      new value appears in telemetry in about a second with no reinstall.
 *
 * How to run it with no Driver Hub, over the USB cable:
 *
 *   adb forward tcp:8001 tcp:8001
 *   adb forward tcp:8002 tcp:8002
 *
 * then open http://localhost:8001 in a browser. Your laptop stays on its normal
 * WiFi -- you do not have to join the Control Hub's network.
 */
@Configurable
@TeleOp(name = "Example: Sloth + Panels Smoke Test", group = "Examples")
public class SlothPanelsSmokeTest extends LinearOpMode {

    // ----------------------------------------------------------------------------------
    // Editable from the Panels "Configurables" panel while the OpMode is running.
    // They must be public static and not final for Panels to see and change them.
    // ----------------------------------------------------------------------------------

    /** Free text echoed to telemetry. Change it in Panels and watch it update live. */
    public static String MESSAGE = "hello from the Control Hub";

    /** How much the counter advances each loop. Try 5 to see it speed up. */
    public static int COUNT_BY = 1;

    // ----------------------------------------------------------------------------------
    // HOT RELOAD MARKER
    //
    // Change this string, then run the "deploySloth" Gradle task. The new value shows
    // up in telemetry in about a second -- no reinstall, no app restart. If it does not
    // change, hot reload is not working; see Examples/README.md.
    //
    // Not final on purpose: the compiler inlines final constants, which would hide the
    // change from a hot reload.
    // ----------------------------------------------------------------------------------

    public static String RELOAD_TAG = "original";

    @Override
    public void runOpMode() {
        // JoinedTelemetry fans one call out to several telemetry sinks. Panels first,
        // then the SDK's own, so the same lines appear in the Panels Telemetry panel
        // and on a Driver Station if one is ever connected.
        Telemetry out = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry);

        out.addData("Status", "Initialized -- no hardware required");
        out.addData("Reload tag", RELOAD_TAG);
        out.update();

        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        long counter = 0;

        while (opModeIsActive()) {
            counter += COUNT_BY;

            out.addData("Reload tag", RELOAD_TAG);
            out.addData("Message", MESSAGE);
            out.addData("Counter", counter);
            out.addData("Count by", COUNT_BY);
            out.addData("Run time", "%.1f s", runtime.seconds());

            // Reading the gamepad needs no hardware, so this works with a controller
            // attached to the laptop through the Panels gamepad panel.
            out.addData("Left stick Y", "%.2f", gamepad1.left_stick_y);
            out.addData("A button", gamepad1.a);

            out.update();

            sleep(50);
        }
    }
}
