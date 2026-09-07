package org.firstinspires.ftc.teamcode.examples;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Wiring check for the 3D-printed dev board. Drives one motor and one servo through a
 * smooth back-and-forth cycle so you can watch, hear and measure whether everything is
 * connected correctly.
 *
 * Expects the "DevBoard" configuration:
 *
 *   Control Hub motor port 0   test_motor   (REV UltraPlanetary HD Hex Motor)
 *   Control Hub servo port 0   test_servo   (REV Smart Robot Servo, REV-41-1097)
 *
 * The motor follows a sine profile: power eases up from zero to MAX_POWER at the middle
 * of each phase, then eases back to zero. Each phase lasts PHASE_SECONDS, and the
 * direction flips between phases. That is gentler on a gearbox than slamming from full
 * forward to full reverse, and it makes a loose coupling or a binding shaft obvious --
 * you hear the load change instead of one abrupt bang.
 *
 * The servo sweeps between SERVO_MIN and SERVO_MAX in step with the motor.
 *
 * WHAT TO LOOK FOR, and what each reading tells you:
 *
 *   Encoder ticks change as the motor turns
 *       If the motor spins but ticks stay at 0, the encoder half of the motor cable is
 *       not seated. That is the single most common dev-board wiring fault, and it is
 *       invisible until you try to use encoders.
 *
 *   Encoder ticks INCREASE while power is positive
 *       If they go negative instead, the motor leads are reversed relative to the
 *       encoder. Fix in code with setDirection, not by re-crimping.
 *
 *   Battery voltage stays well above about 11 V
 *       A big dip when the motor starts means a weak battery or a poor power connection.
 *
 * Everything below is editable live from the Panels Configurables panel, including the
 * two enable flags -- turn one device off to isolate the other while chasing a fault.
 *
 * To run with no Driver Hub, over the USB cable:
 *
 *   adb forward tcp:8001 tcp:8001
 *   adb forward tcp:8002 tcp:8002
 *
 * then open http://localhost:8001 and use the OpModes Control panel.
 */
@Configurable
@TeleOp(name = "Dev Board: Motor + Servo Test", group = "Examples")
public class DevBoardMotorServoTest extends LinearOpMode {

    /** Peak motor power. Starts conservative on purpose; raise it once wiring is proven. */
    public static double MAX_POWER = 0.4;

    /** Seconds per phase. One phase ramps up and back down in a single direction. */
    public static double PHASE_SECONDS = 5.0;

    /** Servo sweep limits, 0..1 across the servo's configured range. */
    public static double SERVO_MIN = 0.25;
    public static double SERVO_MAX = 0.75;

    /** Turn either device off to isolate the other while diagnosing. */
    public static boolean MOTOR_ENABLED = true;
    public static boolean SERVO_ENABLED = true;

    /** Change this and run deploySloth to confirm hot reload. */
    public static String RELOAD_TAG = "original";

    @Override
    public void runOpMode() {
        Telemetry out = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry);

        // Look both devices up before waitForStart so a wiring or naming problem shows
        // up at Init, not several seconds into a run with hardware already moving.
        DcMotorEx motor = null;
        String motorStatus;
        try {
            motor = hardwareMap.get(DcMotorEx.class, "test_motor");
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorStatus = "found, encoder reset to 0";
        } catch (Exception e) {
            motorStatus = "NOT FOUND -- check the config has 'test_motor' on port 0";
        }

        Servo servo = null;
        String servoStatus;
        try {
            servo = hardwareMap.get(Servo.class, "test_servo");
            servoStatus = "found";
        } catch (Exception e) {
            servoStatus = "NOT FOUND -- check the config has 'test_servo' on port 0";
        }

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        out.addData("Reload tag", RELOAD_TAG);
        out.addData("Motor (test_motor)", motorStatus);
        out.addData("Servo (test_servo)", servoStatus);
        out.addData("Battery", "%.2f V", battery.getVoltage());
        out.addLine("");
        out.addLine("Make sure the board is clear and the motor shaft is free.");
        out.addLine("Press Start to begin the sweep. Stop halts everything.");
        out.update();

        waitForStart();

        ElapsedTime phaseTimer = new ElapsedTime();
        int phase = 0;

        while (opModeIsActive()) {
            // Normalised position within this phase, 0..1.
            double t = phaseTimer.seconds() / PHASE_SECONDS;
            if (t >= 1.0) {
                phase++;
                phaseTimer.reset();
                t = 0.0;
            }

            boolean forward = (phase % 2) == 0;

            // sin() eases from 0 up to 1 and back to 0 across the phase, so the motor
            // accelerates and decelerates instead of stepping.
            double shape = Math.sin(Math.PI * t);
            double power = MAX_POWER * shape * (forward ? 1.0 : -1.0);

            if (motor != null && MOTOR_ENABLED) {
                motor.setPower(power);
            } else if (motor != null) {
                motor.setPower(0);
            }

            // Sweep the servo across its range in step with the motor phase, and back
            // again on the reverse phase.
            double servoPos = forward
                    ? SERVO_MIN + (SERVO_MAX - SERVO_MIN) * t
                    : SERVO_MAX - (SERVO_MAX - SERVO_MIN) * t;
            if (servo != null && SERVO_ENABLED) {
                servo.setPosition(servoPos);
            }

            out.addData("Reload tag", RELOAD_TAG);
            out.addData("Phase", "%d (%s)", phase, forward ? "FORWARD" : "REVERSE");
            out.addData("Phase progress", "%.0f%%", t * 100);
            out.addData("Motor power", "%+.3f", motor != null && MOTOR_ENABLED ? power : 0.0);

            if (motor != null) {
                out.addData("Encoder ticks", motor.getCurrentPosition());
                out.addData("Encoder velocity", "%.0f ticks/s", motor.getVelocity());
                out.addData("Motor current", "%.2f A",
                        motor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS));
            } else {
                out.addData("Motor", motorStatus);
            }

            if (servo != null && SERVO_ENABLED) {
                out.addData("Servo position", "%.3f", servoPos);
            } else {
                out.addData("Servo", servo == null ? servoStatus : "disabled");
            }

            out.addData("Battery", "%.2f V", battery.getVoltage());
            out.update();

            sleep(20);
        }

        // Always leave the hardware safe, however the OpMode ended.
        if (motor != null) {
            motor.setPower(0);
        }
    }
}
