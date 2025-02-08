package org.firstinspires.ftc.team22257.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "autoMain")
public class autoMain extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight, UAMotor, LAMotor;
    private Servo ClawServo;
    private CRServo IntakeServo;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        // Define Chassis Motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set Motor Direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Put run blocks here.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Define Arm Motors
        UAMotor = hardwareMap.get(DcMotor.class, "elbow");
        LAMotor = hardwareMap.get(DcMotor.class, "wrist");

        // Set Motor Direction
        UAMotor.setDirection(DcMotor.Direction.FORWARD);
        LAMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set Zero Power Behavior
        UAMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LAMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop and reset motor encoders
        UAMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LAMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Define Claw/Intake Servos
        ClawServo = hardwareMap.get(Servo.class, "claw");
        IntakeServo = hardwareMap.get(CRServo.class, "intake");
        IntakeServo.setDirection(CRServo.Direction.REVERSE);

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            ClawServo.setPosition(1.0);
            UAMotor.setTargetPosition(300);
            UAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
            frontLeft.setPower(0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
            sleep(3000);
            frontLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(-0.5);
            sleep(500);
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            UAMotor.setTargetPosition(0);
            UAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
