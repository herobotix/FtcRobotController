package org.firstinspires.ftc.team00000.teleop;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Main", group="teleop")
//@Disabled
public class Main extends LinearOpMode {
    private DcMotor FLMotor, BLMotor, FRMotor, BRMotor, UAMotor, LAMotor;
    private Servo ClServo;
    private CRServo InServo;
    private IMU imu;
    private boolean ClawOpen, ClawChange;

    public void runOpMode() {
        setupChassis();
        setupActuators();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.start) { imu.resetYaw(); }
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double LSx = gamepad1.left_stick_x, LSy = gamepad1.left_stick_y;
            double RSx = gamepad1.right_stick_x, RSy = gamepad1.right_stick_y;
            double rLSx = -LSx*Math.cos(-botHeading) - LSy*Math.sin(-botHeading);
            double rLSy = -LSx*Math.sin(-botHeading) + LSy*Math.cos(-botHeading);
            double LT =  gamepad1.left_trigger, RT = gamepad1.right_trigger;

            double MaxPwr=Math.max(Math.abs(LSy)+Math.abs(LSx)+Math.abs(RSx), 1);
            FLMotor.setPower((rLSy-rLSx-RSx)/MaxPwr);
            BLMotor.setPower((rLSy+rLSx-RSx)/MaxPwr);
            FRMotor.setPower((rLSy+rLSx+RSx)/MaxPwr);
            BRMotor.setPower((rLSy-rLSx+RSx)/MaxPwr);
            
            //Raise and lower the Arm
            UAMotor.setPower(-RSy);

            //Raise and lower the Intake Arm
            LAMotor.setPower(LT > 0 ? LT : (RT > 0 ? -RT : 0.0));

            //Open and Close Claw
            if(gamepad1.a && ClawChange) {
                if(ClawOpen) {
                    ClServo.setPosition(0.7);
                    ClawOpen=false;
                } else{
                    ClServo.setPosition(0.4);
                    ClawOpen=true;
                }
                ClawChange=false;
            } else if (!gamepad1.a && !ClawChange) ClawChange=true;
            
            if(gamepad1.left_bumper) InServo.setPower(-1);
            else if(gamepad1.right_bumper) InServo.setPower(1);
            else InServo.setPower(0);

            telemetry.addData("GamePad1", teleText("GamePad1"));
            telemetry.addData("GamePad2", teleText("GamePad2"));
            telemetry.update();
        }
    }

    public void setupChassis() {
        // Initialize the hardware motor variables.
        FLMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");

        // Stop and reset motor encoders
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Setup to use motor encoders
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU and set directions of Logo and USB port
        imu = hardwareMap.get(IMU.class,"IMU");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
    }

    public void setupActuators() {
        // Initialize the hardware variables.
        UAMotor = hardwareMap.get(DcMotor.class, "UpperArmMotor");
        LAMotor = hardwareMap.get(DcMotor.class, "LowerArmMotor");
        ClServo = hardwareMap.get(Servo.class, "ClawServo");
        InServo = hardwareMap.get(CRServo.class, "IntakeServo");

        // Initialize Claw State
        ClawOpen=false;
        ClawChange=false;

        // Initialize Intake State
        InServo.setDirection(CRServo.Direction.REVERSE);
    }

    public String teleText(@NonNull String pick) {
        String txt = "";
        switch (pick) {
            case "GamePad1":
                txt += "\n\tLeftStick:\tbtn: " + (gamepad1.left_stick_button?1:0) + "\tx: " + _p(gamepad1.left_stick_x) + "\ty: " + _p(gamepad1.left_stick_y);
                txt += "\n\tRightStick:\tbtn: " + (gamepad1.right_stick_button?1:0) + "\tx: " + _p(gamepad1.right_stick_x) + "\ty: " + _p(gamepad1.right_stick_y);
                txt += "\n\tLBump: " + (gamepad1.left_bumper?1:0) + "\tRBump: " + (gamepad1.right_bumper?1:0) +
                             "\tLTrig: " + _p(gamepad1.left_trigger) + "\tRTrig: " + _p(gamepad1.right_trigger);
                txt += "\n\ta: " + (gamepad1.a?1:0) + "\tb: " + (gamepad1.b?1:0) + "\ty: " + (gamepad1.y?1:0) +
                             "\tx: " + (gamepad1.x?1:0) + "\tback: " + (gamepad1.back?1:0) + "\tstart: " + (gamepad1.start?1:0);
                txt += "\n\tDpad:\tL: " + (gamepad1.dpad_left?1:0) + "\tR: " + (gamepad1.dpad_right?1:0) +
                                      "\tU: " + (gamepad1.dpad_up?1:0) + "\tD: " + (gamepad1.dpad_down?1:0);
                break;
            case "GamePad2":
                txt += "\n\tLeftStick:\tbtn: " + (gamepad2.left_stick_button?1:0) + "\tx: " + _p(gamepad2.left_stick_x) + "\ty: " + _p(gamepad2.left_stick_y);
                txt += "\n\tRightStick:\tbtn: " + (gamepad2.right_stick_button?1:0) + "\tx: " + _p(gamepad2.right_stick_x) + "\ty: " + _p(gamepad2.right_stick_y);
                txt += "\n\tLBump: " + (gamepad2.left_bumper?1:0) + "\tRBump: " + (gamepad2.right_bumper?1:0) +
                        "\tLTrig: " + _p(gamepad2.left_trigger) + "\tRTrig: " + _p(gamepad2.right_trigger);
                txt += "\n\ta: " + (gamepad2.a?1:0) + "\tb: " + (gamepad2.b?1:0) + "\ty: " + (gamepad2.y?1:0) +
                        "\tx: " + (gamepad2.x?1:0) + "\tback: " + (gamepad2.back?1:0) + "\tstart: " + (gamepad2.start?1:0);
                txt += "\n\tDpad:\tL: " + (gamepad2.dpad_left?1:0) + "\tR: " + (gamepad2.dpad_right?1:0) +
                        "\tU: " + (gamepad2.dpad_up?1:0) + "\tD: " + (gamepad2.dpad_down?1:0);
                break;
        }
        return txt;

    }

    public String _p(double val) {
        return String.format("%.3f",val);
    }

}