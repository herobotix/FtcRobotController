package org.firstinspires.ftc.teamcode.examples;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Sensors driving actuators on the dev board. Two independent loops the students can
 * see, hear and reason about:
 *
 *   Distance sensor -> motor speed
 *       Put your hand close and the motor slows down. Take it away and it speeds up.
 *       Distance is clamped between NEAR_CM and FAR_CM and mapped onto 0..MAX_POWER.
 *
 *   Touch sensor -> servo
 *       Hold the button and the servo sweeps back and forth. Let go and it parks at
 *       SERVO_HOME. A REV Smart Robot Servo is positional, so it sweeps rather than
 *       spins continuously.
 *
 * Expects the "DevBoard" configuration:
 *
 *   Control Hub motor port 0     test_motor      REV UltraPlanetary HD Hex Motor
 *   Control Hub servo port 0     test_servo      REV Smart Robot Servo (REV-41-1097)
 *   Control Hub I2C bus 0        test_distance   REV 2m Distance Sensor (addr 0x29)
 *   Control Hub digital 0-1      test_touch      REV Touch Sensor
 *
 * NOTE ON THE TOUCH SENSOR PORT. A REV Touch Sensor plugged into the "0,1" digital port
 * is wired to pin **1** -- the 4-wire JST cable leaves pin 0 disconnected. So it is
 * configured on channel 1, not 0. Configuring it on 0 gives a sensor that never reads,
 * with no error to tell you why. This catches people every season.
 *
 * SAFETY. With nothing in front of the distance sensor the reading is far away, which
 * means the motor runs at MAX_POWER. That is intended -- "no hand, full speed" -- but it
 * means the motor starts moving as soon as you press Start. MAX_POWER is 0.4 and the
 * OpMode stops itself after RUN_SECONDS. The hub's power switch is the real emergency
 * stop; do not rely on the Panels Stop button, which can be swallowed by the UI.
 *
 * To run with no Driver Hub, over the USB cable:
 *
 *   adb forward tcp:8001 tcp:8001
 *   adb forward tcp:8002 tcp:8002
 *
 * then open http://localhost:8001 and use the OpModes Control panel.
 */
@Configurable
@TeleOp(name = "Dev Board: Sensor Control Test", group = "Examples")
public class DevBoardSensorTest extends LinearOpMode {

    /** At or below this distance the motor is stopped. */
    public static double NEAR_CM = 5.0;

    /** At or above this distance the motor runs at MAX_POWER. */
    public static double FAR_CM = 40.0;

    /**
     * Readings at or above this are treated as "nothing in view".
     *
     * The REV 2m sensor does not return NaN when it sees nothing -- it returns its
     * sentinel value, about 819 cm (8190 mm), which is far beyond its 2 m range. Without
     * this check that sentinel looks like a real measurement.
     */
    public static double MAX_VALID_CM = 250.0;

    /** Ceiling on motor power. Conservative on purpose. */
    public static double MAX_POWER = 0.4;

    /** Servo sweep limits and resting position, 0..1. */
    public static double SERVO_MIN = 0.25;
    public static double SERVO_MAX = 0.75;
    public static double SERVO_HOME = 0.5;

    /** Seconds for one full servo sweep out and back while the button is held. */
    public static double SWEEP_SECONDS = 1.5;

    /** Stop automatically after this many seconds. 0 means run until Stop is pressed. */
    public static double RUN_SECONDS = 30.0;

    /** Change this and run deploySloth to confirm hot reload. */
    public static String RELOAD_TAG = "original";

    @Override
    public void runOpMode() {
        Telemetry out = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry);

        // Look everything up at Init so a wiring or naming fault is visible before
        // anything moves.
        DcMotorEx motor = null;
        Servo servo = null;
        DistanceSensor distance = null;
        TouchSensor touch = null;
        String motorStatus, servoStatus, distanceStatus, touchStatus;

        try {
            motor = hardwareMap.get(DcMotorEx.class, "test_motor");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorStatus = "found";
        } catch (Exception e) {
            motorStatus = "NOT FOUND (test_motor)";
        }
        try {
            servo = hardwareMap.get(Servo.class, "test_servo");
            servoStatus = "found";
        } catch (Exception e) {
            servoStatus = "NOT FOUND (test_servo)";
        }
        try {
            distance = hardwareMap.get(DistanceSensor.class, "test_distance");
            distanceStatus = String.format("found, reading %.1f cm",
                    distance.getDistance(DistanceUnit.CM));
        } catch (Exception e) {
            distanceStatus = "NOT FOUND (test_distance) -- check I2C bus 0";
        }
        try {
            touch = hardwareMap.get(TouchSensor.class, "test_touch");
            touchStatus = "found, currently " + (touch.isPressed() ? "PRESSED" : "released");
        } catch (Exception e) {
            touchStatus = "NOT FOUND (test_touch) -- must be on digital channel 1, not 0";
        }

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        out.addData("Reload tag", RELOAD_TAG);
        out.addData("Motor", motorStatus);
        out.addData("Servo", servoStatus);
        out.addData("Distance sensor", distanceStatus);
        out.addData("Touch sensor", touchStatus);
        out.addData("Battery", "%.2f V", battery.getVoltage());
        out.addLine("");
        out.addLine("Press the touch sensor now to check it before starting.");
        out.addLine("On Start the motor runs -- keep the shaft clear.");
        out.update();

        // Let the button be tested during Init, while nothing is moving.
        while (opModeInInit()) {
            if (touch != null) {
                out.addData("Reload tag", RELOAD_TAG);
                out.addData("Touch sensor", touch.isPressed() ? "PRESSED" : "released");
                if (distance != null) {
                    double d = distance.getDistance(DistanceUnit.CM);
                    out.addData("Distance", d >= MAX_VALID_CM
                            ? String.format("nothing in view (raw %.0f cm)", d)
                            : String.format("%.1f cm", d));
                }
                out.addLine("Press Start when ready.");
                out.update();
            }
            sleep(50);
        }

        ElapsedTime runTimer = new ElapsedTime();
        ElapsedTime sweepTimer = new ElapsedTime();
        boolean wasPressed = false;

        while (opModeIsActive()) {
            if (RUN_SECONDS > 0 && runTimer.seconds() >= RUN_SECONDS) {
                break;
            }

            // ---- distance sensor drives motor speed ----------------------------------
            double cm = Double.NaN;
            double power = 0;
            String distanceNote = "no sensor";
            if (distance != null) {
                cm = distance.getDistance(DistanceUnit.CM);
                if (Double.isNaN(cm) || cm >= MAX_VALID_CM) {
                    // Nothing in view: NaN on some sensors, the ~819 cm sentinel on this
                    // one. Either way it means "no obstacle", the far end of the scale.
                    power = MAX_POWER;
                    distanceNote = "nothing in view -> full speed";
                } else {
                    double span = Math.max(0.001, FAR_CM - NEAR_CM);
                    double norm = (cm - NEAR_CM) / span;
                    norm = Math.max(0.0, Math.min(1.0, norm));
                    power = MAX_POWER * norm;
                    distanceNote = String.format("%.0f%% of range", norm * 100);
                }
            }
            if (motor != null) {
                motor.setPower(power);
            }

            // ---- touch sensor drives the servo ---------------------------------------
            boolean pressed = touch != null && touch.isPressed();
            if (pressed && !wasPressed) {
                sweepTimer.reset();      // start a fresh sweep on each new press
            }
            wasPressed = pressed;

            double servoPos = SERVO_HOME;
            if (pressed) {
                // Triangle wave: out to SERVO_MAX and back to SERVO_MIN, repeating.
                double phase = (sweepTimer.seconds() % SWEEP_SECONDS) / SWEEP_SECONDS;
                double tri = phase < 0.5 ? (phase * 2) : (2 - phase * 2);
                servoPos = SERVO_MIN + (SERVO_MAX - SERVO_MIN) * tri;
            }
            if (servo != null) {
                servo.setPosition(servoPos);
            }

            out.addData("Reload tag", RELOAD_TAG);
            out.addData("Elapsed", "%.1f / %.0f s", runTimer.seconds(), RUN_SECONDS);
            out.addLine("");
            out.addData("Distance", (Double.isNaN(cm) || cm >= MAX_VALID_CM)
                    ? String.format("nothing in view (raw %.0f cm)", cm)
                    : String.format("%.1f cm", cm));
            out.addData("  -> motor power", "%.3f  (%s)", power, distanceNote);
            if (motor != null) {
                out.addData("  encoder velocity", "%.0f ticks/s", motor.getVelocity());
            }
            out.addLine("");
            out.addData("Touch", pressed ? "PRESSED -- servo sweeping" : "released -- servo parked");
            out.addData("  -> servo position", "%.3f", servoPos);
            out.addLine("");
            out.addData("Battery", "%.2f V", battery.getVoltage());
            out.update();

            sleep(20);
        }

        if (motor != null) {
            motor.setPower(0);
        }
        if (servo != null) {
            servo.setPosition(SERVO_HOME);
        }
    }
}
