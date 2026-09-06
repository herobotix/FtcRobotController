package org.firstinspires.ftc.teamcode.examples;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.FeedbackType;
import dev.nextftc.control.feedback.PIDElement;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Proves that Pedro Pathing, NextFTC and Limelight all coexist with Sloth hot reload
 * and FTControl Panels in one module.
 *
 * Runs on a bare Control Hub with nothing plugged in. The Pedro and NextFTC checks do
 * real computation -- they are not just imports -- because a library can compile and
 * still fail to load on the robot if something went wrong when it was dexed. The
 * Limelight check is the only one that needs hardware, and it reports its absence
 * instead of crashing.
 *
 * To run with no Driver Hub:
 *
 *   adb forward tcp:8001 tcp:8001
 *   adb forward tcp:8002 tcp:8002
 *
 * then open http://localhost:8001 and use the OpModes Control panel.
 *
 * This is a diagnostic, not a template to copy for robot code. For how these libraries
 * are actually used, look at a team's module.
 */
@Configurable
@TeleOp(name = "Example: Library Integration Test", group = "Examples")
public class LibraryIntegrationTest extends LinearOpMode {

    /** Name to look for in the robot configuration. Editable from Panels. */
    public static String LIMELIGHT_NAME = "limelight";

    /** Proportional gain for the NextFTC PID check. Editable from Panels. */
    public static double TEST_KP = 0.01;

    /** Change this and run deploySloth to confirm hot reload still works. */
    public static String RELOAD_TAG = "original";

    @Override
    public void runOpMode() {
        Telemetry out = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry);

        // --- Pedro Pathing: pure geometry, no hardware ----------------------------
        // Build a line between two poses and measure it. If Pedro did not load, this
        // throws and we report it rather than failing silently.
        String pedroResult;
        try {
            Pose start = new Pose(0, 0, 0);
            Pose end = new Pose(24, 12, Math.toRadians(90));
            BezierLine line = new BezierLine(start, end);
            double length = line.approximateLength();
            Pose midpoint = line.getPose(0.5);
            pedroResult = String.format(
                    "OK  length=%.2f in, midpoint=(%.1f, %.1f)",
                    length, midpoint.getX(), midpoint.getY());
        } catch (Throwable t) {
            pedroResult = "FAILED  " + t.getClass().getSimpleName() + ": " + t.getMessage();
        }

        // --- NextFTC: run a PID calculation, no hardware ---------------------------
        String nextFtcResult;
        try {
            PIDElement pid = new PIDElement(FeedbackType.POSITION, TEST_KP, 0.0, 0.0);
            // Goal of 100 ticks, currently at 0, so the output should be kP * 100.
            KineticState error = new KineticState(100, 0, 0);
            double output = pid.calculate(error);
            nextFtcResult = String.format("OK  pid(kP=%.3f, error=100) = %.3f", TEST_KP, output);
        } catch (Throwable t) {
            nextFtcResult = "FAILED  " + t.getClass().getSimpleName() + ": " + t.getMessage();
        }

        // --- Limelight: the one check that genuinely needs hardware ----------------
        // Part of the FTC SDK, so the classes are always present. Whether a camera is
        // actually attached and configured is a separate question.
        Limelight3A limelight = null;
        String limelightResult;
        try {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
            limelightResult = "found in configuration as '" + LIMELIGHT_NAME + "'";
        } catch (Throwable t) {
            limelightResult = "not in configuration (expected with no camera attached)"
                    + " -- SDK classes loaded fine";
        }

        out.addData("Reload tag", RELOAD_TAG);
        out.addData("Pedro Pathing", pedroResult);
        out.addData("NextFTC", nextFtcResult);
        out.addData("Limelight", limelightResult);
        out.addLine("Press Start to poll the Limelight, if one is attached.");
        out.update();

        waitForStart();

        if (limelight != null) {
            limelight.start();
        }

        while (opModeIsActive()) {
            out.addData("Reload tag", RELOAD_TAG);
            out.addData("Pedro Pathing", pedroResult);
            out.addData("NextFTC", nextFtcResult);

            if (limelight == null) {
                out.addData("Limelight", limelightResult);
            } else {
                LLStatus status = limelight.getStatus();
                out.addData("Limelight temp", "%.1f C", status.getTemp());
                out.addData("Limelight pipeline", status.getPipelineIndex());
                out.addData("Limelight fps", status.getFps());
            }

            out.update();
            sleep(100);
        }

        if (limelight != null) {
            limelight.stop();
        }
    }
}
