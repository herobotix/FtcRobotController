package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Main", group="teleop")
//@Disabled
public class Main extends LinearOpMode {
    private DcMotor FLMotor, BLMotor, FRMotor, BRMotor, UAMotor, LAMotor;
    private Servo ClawServo;
    private CRServo IntakeServo;
    private IMU imu;
    private boolean ClawOpen, ClawChange;

    // UpperArm, LowerArm target positions for each state
    private static final int UA_POS_INIT = 300;
    private static final int UA_POS_INTAKE = 450;
    private static final int UA_POS_WALL_GRAB = 1100;
    private static final int UA_POS_WALL_UNHOOK = 1700;
    private static final int UA_POS_HOVER_HIGH = 2600;
    private static final int UA_POS_CLIP_HIGH = 2100;
    private static final int UA_POS_LOW_BASKET = 2500;
    private static final int LA_POS_INIT = 0;
    private static final int LA_POS_SAMPLE = 270;
    private static final int LA_POS_SPEC = 10;

    // Claw positions for each state
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
    private int targetUpperArm = 0;
    private int targetLowerArm = 0;

    @Override
    public void runOpMode() {
        setupChassis();
        setupActuators();

        double botHeading=0, LSx=0, LSy=0, RSx=0, RSy=0, rLSx=0, rLSy=0, LT=0, RT=0, MaxPwr=1;
        String tele="";

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            tele="";
            // Set Upper/Lower Arm Positions based on state.
            switch (currentState) {
                case INIT:
                    targetUpperArm = UA_POS_INIT;
                    targetLowerArm = LA_POS_INIT;
                    tele += "\tINIT";
                    break;
                case INTAKE:
                    targetUpperArm = UA_POS_INTAKE;
                    targetLowerArm = LA_POS_SAMPLE;
                    tele += "\tINTAKE";
                    break;
                case WALL_GRAB:
                    targetUpperArm = UA_POS_WALL_GRAB;
                    targetLowerArm = LA_POS_SPEC;
                    tele += "\tWALL_GRAB";
                    break;
                case WALL_UNHOOK:
                    targetUpperArm = UA_POS_WALL_UNHOOK;
                    targetLowerArm = LA_POS_SPEC;
                    tele += "\tWALL_UNHOOK";
                    break;
                case HOVER_HIGH:
                    targetUpperArm = UA_POS_HOVER_HIGH;
                    targetLowerArm = LA_POS_SPEC;
                    tele += "\tHOVER_HIGH";
                    break;
                case CLIP_HIGH:
                    targetUpperArm = UA_POS_CLIP_HIGH;
                    targetLowerArm = LA_POS_SPEC;
                    tele += "\tCLIP_HIGH";
                    break;
                case LOW_BASKET:
                    targetUpperArm = UA_POS_LOW_BASKET;
                    targetLowerArm = LA_POS_SAMPLE;
                    tele += "\tLOW_BASKET";
                    break;
                case MANUAL:
                    tele += "\tMANUAL";
                    break;
            }

            // Set currentState based on user input.
            if (gamepad2.a) {
                currentState = RobotState.INTAKE;
            } else if (gamepad2.b && !lastGrab) {
                if(currentState == RobotState.WALL_GRAB) { currentState = RobotState.WALL_UNHOOK; }
                else { currentState = RobotState.WALL_GRAB; }
            } else if (gamepad2.y && !lastHook) {
                if(currentState == RobotState.HOVER_HIGH){ currentState = RobotState.CLIP_HIGH; }
                else { currentState = RobotState.HOVER_HIGH; }
            } else if (gamepad2.x) {
                currentState = RobotState.LOW_BASKET;
            } else if (gamepad2.left_bumper) {
                currentState = RobotState.INIT;
            } else if (gamepad2.dpad_up){ //manual control
                currentState = RobotState.MANUAL;
                targetUpperArm += 10;
            } else if (gamepad2.dpad_down){
                currentState = RobotState.MANUAL;
                targetUpperArm -= 10;
            } else if (gamepad2.dpad_left){
                currentState = RobotState.MANUAL;
                targetLowerArm += 1;
            } else if (gamepad2.dpad_right){
                currentState = RobotState.MANUAL;
                targetLowerArm -= 1;
            }

            lastGrab = gamepad2.b;
            lastHook = gamepad2.y;

            // Toggle claw position when right_bumper is pressed
            if (gamepad2.right_bumper && !lastBump) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    ClawServo.setPosition(CLAW_OPEN_POSITION);
                } else {
                    ClawServo.setPosition(CLAW_CLOSED_POSITION);
                }
            }
            lastBump = gamepad2.right_bumper;

            // Control intake servo with triggers
            if (gamepad2.right_trigger>0.1) {
                IntakeServo.setPower(1.0);
            } else if (gamepad2.left_trigger>0.1) {
                IntakeServo.setPower(-1.0);
            } else {
                IntakeServo.setPower(0);
            }

            if (gamepad1.start) { imu.resetYaw(); }
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            LSx = gamepad1.left_stick_x;
            LSy = gamepad1.left_stick_y;
            RSx = gamepad1.right_stick_x;
            RSy = gamepad1.right_stick_y;
            rLSx = LSx*Math.cos(botHeading) - LSy*Math.sin(botHeading);
            rLSy = LSx*Math.sin(botHeading) + LSy*Math.cos(botHeading);
            LT =  gamepad1.left_trigger;
            RT = gamepad1.right_trigger;
            MaxPwr=Math.max(Math.abs(LSy)+Math.abs(LSx)+Math.abs(RSx), 1);

            FLMotor.setPower((-rLSy+rLSx+RSx)/MaxPwr);
            BLMotor.setPower((-rLSy-rLSx+RSx)/MaxPwr);
            FRMotor.setPower((-rLSy-rLSx-RSx)/MaxPwr);
            BRMotor.setPower((-rLSy+rLSx-RSx)/MaxPwr);

            UAMotor.setTargetPosition(targetUpperArm);
            UAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LAMotor.setTargetPosition(targetLowerArm);
            LAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            UAMotor.setPower(1);
            LAMotor.setPower(1);

            // Send telemetry data to the driver station
            tele += "\n\tHeading: " + _p(botHeading) + "\tClaw: " + (clawOpen ? "Open" : "Closed");
            tele += "\n\tUpperArm: Pos: " + _p(UAMotor.getCurrentPosition()) + "/" + _p(targetUpperArm) + ", Pwr: " + _p(UAMotor.getPower());
            tele += "\n\tLowerArm: Pos: " + _p(LAMotor.getCurrentPosition()) + "/" + _p(targetLowerArm) + ", Pwr: " + _p(LAMotor.getPower());
            telemetry.addData("State",tele);

            /*
            //Raise and lower the Arm
            UAMotor.setPower(-RSy);

            //Raise and lower the Intake Arm
            LAMotor.setPower(LT > 0 ? LT : (RT > 0 ? -RT : 0.0));

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
            */

            telemetry.addData("GamePad1", teleText("GamePad1"));
            telemetry.addData("GamePad2", teleText("GamePad2"));
            telemetry.update();
        }
    }

    public void setupChassis() {
        // Map the hardware motor variables.
        FLMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");

        // Set Motor Direction
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set Zero Power Behavior
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        // Map the hardware actuator variables.
        UAMotor = hardwareMap.get(DcMotor.class, "UpperArmMotor");
        LAMotor = hardwareMap.get(DcMotor.class, "LowerArmMotor");
        ClawServo = hardwareMap.get(Servo.class, "ClawServo");
        IntakeServo = hardwareMap.get(CRServo.class, "IntakeServo");

        // Set Zero Power Behavior
        UAMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LAMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Claw State
        //ClawOpen=false;
        //ClawChange=false;

        // Initialize Intake State
        IntakeServo.setDirection(CRServo.Direction.REVERSE);
    }

    public String teleText(String pick) {
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