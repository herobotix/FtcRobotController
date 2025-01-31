package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="StarterBot_V1_Java", group="Linear Opmode")
public class StarterBot2025TeleOpJava extends LinearOpMode {

    // Declare OpMode members.
    private DcMotorEx FLMotor,BLMotor,FRMotor,BRMotor;
    private DcMotor arm = null;
    private DcMotor wrist = null;
    private Servo claw = null;
    private CRServo intake = null;


    // Arm and Wrist target positions for each state
    private static final int ARM_POSITION_INIT = 300;
    private static final int ARM_POSITION_INTAKE = 300;
    private static final int ARM_POSITION_WALL_GRAB = 1000;
    private static final int ARM_POSITION_WALL_UNHOOK = 1700;
    private static final int ARM_POSITION_HOVER_HIGH = 2600;
    private static final int ARM_POSITION_CLIP_HIGH = 2100;
    private static final int ARM_POSITION_LOW_BASKET = 2500;


    private static final int WRIST_POSITION_INIT = 0;
    private static final int WRIST_POSITION_SAMPLE = 270;
    private static final int WRIST_POSITION_SPEC = 10;



    // Claw positions
    private static final double CLAW_OPEN_POSITION = 0.55;
    private static final double CLAW_CLOSED_POSITION = 0.7;

    // Enum for state machine
    private enum RobotState {
        INIT,
        INTAKE,
        WALL_GRAB,
        WALL_UNHOOK,
        HOVER_HIGH,
        CLIP_HIGH,
        LOW_BASKET,
        MANUAL
    }

    // Initial state
    private RobotState currentState = RobotState.INIT;

    // Claw toggle state
    private boolean clawOpen = true;
    private boolean lastBump = false;
    private boolean lastHook = false;
    private boolean lastGrab = false;

    //target position
    private int targetArm = 0;
    private int targetWrist = 0;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        FLMotor  = hardwareMap.get(DcMotorEx.class, "FLMotor");
        BLMotor  = hardwareMap.get(DcMotorEx.class, "BLMotor");
        BRMotor  = hardwareMap.get(DcMotorEx.class, "BRMotor");
        FRMotor  = hardwareMap.get(DcMotorEx.class, "FRMotor");
        arm = hardwareMap.get(DcMotor.class, "ArmMotor");
        wrist = hardwareMap.get(DcMotor.class, "WristMotor");
        claw = hardwareMap.get(Servo.class, "ClawServo");
        intake = hardwareMap.get(CRServo.class, "IntakeServo");
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu.initialize(parameters);

        // Stop and reset encoders
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set zero power behavior
        wrist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // State machine logic
            switch (currentState) {
                case INIT:
                    targetArm = ARM_POSITION_INIT;
                    targetWrist = WRIST_POSITION_INIT;
                    telemetry.addData("State", "INIT");
                    break;
                case INTAKE:
                    targetArm = ARM_POSITION_INTAKE;
                    targetWrist = WRIST_POSITION_SAMPLE;
                    telemetry.addData("State", "INTAKE");
                    break;

                case WALL_GRAB:
                    targetArm = ARM_POSITION_WALL_GRAB;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "WALL_GRAB");
                    break;

                case WALL_UNHOOK:
                    targetArm = ARM_POSITION_WALL_UNHOOK;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "WALL_UNHOOK");
                    break;

                case HOVER_HIGH:
                    targetArm = ARM_POSITION_HOVER_HIGH;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "HOVER_HIGH");
                    break;

                case CLIP_HIGH:
                    targetArm = ARM_POSITION_CLIP_HIGH;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "CLIP_HIGH");
                    break;
                case LOW_BASKET:
                    targetArm = ARM_POSITION_LOW_BASKET;
                    targetWrist = WRIST_POSITION_SAMPLE;
                    telemetry.addData("State", "LOW_BASKET");
                    break;
                case MANUAL:
                    telemetry.addData("State", "MANUAL");
                    break;
            }


            // Handle state transitions based on gamepad input
            if (gamepad1.a) {
                currentState = RobotState.INTAKE;
            } else if (gamepad1.b && !lastGrab) {
                if(currentState == RobotState.WALL_GRAB){
                    currentState = RobotState.WALL_UNHOOK;
                }else{
                    currentState = RobotState.WALL_GRAB;
                }
            } else if (gamepad1.y && !lastHook) {
                if(currentState == RobotState.HOVER_HIGH){
                    currentState = RobotState.CLIP_HIGH;
                }else{
                    currentState = RobotState.HOVER_HIGH;
                }
            } else if (gamepad1.x) {
                currentState = RobotState.LOW_BASKET;
            } else if (gamepad1.left_bumper) {
                currentState = RobotState.INIT;
            } else if (gamepad1.dpad_up){ //manual control
                currentState = RobotState.MANUAL;
                targetArm += 10;
            } else if (gamepad1.dpad_down){
                currentState = RobotState.MANUAL;
                targetArm -= 10;
            } else if (gamepad1.dpad_left){
                currentState = RobotState.MANUAL;
                targetWrist += 1;
            } else if (gamepad1.dpad_right){
                currentState = RobotState.MANUAL;
                targetWrist -= 1;
            }

            lastGrab = gamepad1.b;
            lastHook = gamepad1.y;

            // Toggle claw position when right_bumper is pressed
            if (gamepad1.right_bumper && !lastBump) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    claw.setPosition(CLAW_OPEN_POSITION);
                } else {
                    claw.setPosition(CLAW_CLOSED_POSITION);
                }
            }
            lastBump = gamepad1.right_bumper;

            // Control intake servo with triggers
            if (gamepad1.right_trigger>0.1) {
                intake.setPower(1.0);
            } else if (gamepad1.left_trigger>0.1) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0);
            }

            //DRIVE Split Arcade
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            //double rotX = x;
            //double rotY = y;
            double rotX = - x * Math.cos(-botHeading) + y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing





            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            FLMotor.setPower(frontLeftPower);
            BLMotor.setPower(backLeftPower);
            FRMotor.setPower(frontRightPower);
            BRMotor.setPower(backRightPower);

            arm.setTargetPosition(targetArm);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setTargetPosition(targetWrist);
            wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            wrist.setPower(1);

            // Send telemetry data to the driver station
            telemetry.addData("Claw Position", clawOpen ? "Open" : "Closed");
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Arm Power", arm.getPower());
            telemetry.addData("Wrist Position", wrist.getCurrentPosition());
            telemetry.addData("Wrist Power", wrist.getPower());
            telemetry.update();
        }
    }
}