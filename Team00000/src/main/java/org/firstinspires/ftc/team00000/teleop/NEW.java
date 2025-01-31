package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team00000.ChassisMecanum;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="NEW", group="teleop")
//@Disabled
public class NEW extends LinearOpMode {
    // Declare the hardware variables.
    private DcMotorEx BLMotor;
    private DcMotorEx BRMotor;
    private DcMotorEx FLMotor;
    private DcMotorEx FRMotor;
    private DcMotorSimple ArmMotor;
    private DcMotorSimple IntakeArmMotor;
    private Servo ClawServo;
    private CRServo IntakeServo;
    private IMU imu;

    // Declare OpMode Variables
    private double LSx, rLSx, LSy, rLSy, RSx;
    private double FRPwr, BRPwr, FLPwr, BLPwr, MaxPwr;
    private double ArmKf;
    private boolean ClawOpen, ClawChange;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map the hardware variables.
        FLMotor  = hardwareMap.get(DcMotorEx.class, "FLMotor");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BLMotor");
        FRMotor  = hardwareMap.get(DcMotorEx.class, "FRMotor");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BRMotor");
        ArmMotor= hardwareMap.get(DcMotorSimple.class, "ArmMotor");
        IntakeArmMotor= hardwareMap.get(DcMotorSimple.class, "IntakeArmMotor");
        ClawServo= hardwareMap.get(Servo.class, "ClawServo");
        IntakeServo= hardwareMap.get(CRServo.class, "IntakeServo");
        imu = hardwareMap.get(IMU.class,"imu");

        // Initialize the hardware variables.
        //     Stop, reset, and enable wheel motor encoders.
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //     Set Claw State
        ClawOpen=false;
        ClawChange=true;

        //     Set Intake State
        IntakeServo.setDirection(CRServo.Direction.REVERSE);

        //     Set IMU directions of Logo and USB port
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.start) { imu.resetYaw(); }
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            LSx=-gamepad1.left_stick_x;
            LSy=gamepad1.left_stick_y;
            rLSx=LSx*Math.cos(-botHeading)-LSy*Math.sin(-botHeading);
            rLSy=LSx*Math.sin(-botHeading)+LSy*Math.cos(-botHeading);
            RSx=-gamepad1.right_stick_x;

            MaxPwr=Math.max(Math.abs(rLSx)+Math.abs(rLSy)+Math.abs(RSx), 1);
            FLPwr=(rLSy+rLSx+RSx)/MaxPwr;
            BLPwr=(rLSy-rLSx+RSx)/MaxPwr;
            FRPwr=(rLSy-rLSx-RSx)/MaxPwr;
            BRPwr=(rLSy+rLSx-RSx)/MaxPwr;

             // Handle state transitions based on gamepad input
            FLMotor.setPower(FLPwr); 
            BLMotor.setPower(BLPwr);
            FRMotor.setPower(FRPwr); 
            BRMotor.setPower(BRPwr);

            //Raise and lower the Arm
            ArmMotor.setPower(-gamepad1.right_stick_y);

            //Raise and lower the Intake Arm
            if(gamepad1.left_trigger>0) IntakeArmMotor.setPower(gamepad1.left_trigger);
            else if(gamepad1.right_trigger>0) IntakeArmMotor.setPower(-gamepad1.right_trigger);
            else IntakeArmMotor.setPower(0.0);

            //Open and Close Claw
            if(gamepad1.a && ClawChange) {
                if(ClawOpen) {
                    ClawServo.setPosition(0.7);
                    ClawOpen=false;
                } else{
                    ClawServo.setPosition(0.4);
                    ClawOpen=true;
                }
                ClawChange=false;
            } else if (!gamepad1.a && !ClawChange) ClawChange=true;

            if(gamepad1.left_bumper) IntakeServo.setPower(-1);
            else if(gamepad1.right_bumper) IntakeServo.setPower(1);
            else IntakeServo.setPower(0);

            //telemetry.update();
        }
    }
}