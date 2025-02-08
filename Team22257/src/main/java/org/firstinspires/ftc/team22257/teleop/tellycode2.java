package org.firstinspires.ftc.team22257.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="tellycode2", group="teleop")
//@Disabled
public class tellycode2 extends LinearOpMode {
    private DcMotor FLMotor, BLMotor, FRMotor, BRMotor, UAMotor, LAMotor;
    private Servo ClawServo;
    private CRServo IntakeServo;
    private IMU imu;

    // UpperArm predefined target positions
    private static final int UA_POS_MIN = 0;
    private static final int UA_POS_MAX = 4650;
    private static final int UA_POS_GET_SPECIMEN = 0;
    private static final int UA_POS_GET_SAMPLE = 230;
    private static final int UA_POS_BL_HOVER = 3500;
    private static final int UA_POS_CL_HOVER = 2700;
    private static final int UA_POS_CL_CLIP = 2100;
    private static final int UA_POS_CH_HOVER = 4500;
    private static final int UA_POS_CH_CLIP = 3900;
    private static final int UA_POS_RL_HOVER = 4400;
    private static final int UA_POS_RL_LOCK = 4300;  //UNTESTED

    // LowerArm predefined target positions
    private static final int LA_POS_MIN = 0;
    private static final int LA_POS_MAX = 230;
    private static final int LA_POS_SPECIMEN = 0;
    private static final int LA_POS_SAMPLE = 215;
    private static final int LA_POS_BL_HOVER = 230;

    // Claw predefined positions
    private static final double CLAW_OPEN_POSITION = 0.5;
    private static final double CLAW_CLOSED_POSITION = 1.0;

    // Enum for state machine
    private enum ArmState {
        HOME,           //Same as GET_SPEC (Used as starting position.)
        GET_SPEC,       //Get Specimen (Upper Arm at lowest state.  Lower Arm contracted.)
        GET_SAMP,       //Get Sample (Upper Arm lifted slightly.  Lower Arm extended.)
        BL_HOVER,       //Low Basket Hover (Upper and Lower Arms extended. Hover Intake above Low Basket.)
        CL_HOVER,       //Low Chamber Hover (Upper Arm extended.  Lower Arm contracted.  Hover specimen above Low Chamber.)
        CL_CLIP,        //Low Chamber Clip (Upper Arm extended.  Lower Arm contracted.  Drop specimen onto Low Chamber.)
        CH_HOVER,       //High Chamber Hover (Upper Arm extended.  Lower Arm contracted.  Hover specimen above High Chamber.)
        CH_CLIP,        //High Chamber Clip (Upper Arm extended.  Lower Arm contracted.  Drop specimen onto High Chamber.)
        RL_HOVER,       //Low Rung Hover (Upper Arm extended.  Lower Arm contracted.  Hover coupler over Low Rung.)
        RL_LOCK,        //Low Rung Lock (Upper Arm extended.  Lower Arm contracted.  Drop coupler over Low Rung.)
        MANUAL          //Allow manual movement of both Upper and Lower Arm.
    }

    @Override
    public void runOpMode() {

        //Default States and Values
        ArmState currentState = ArmState.HOME;
        boolean hold1Y = false, hold1RT = false, hold1S = false;
        boolean hold2A = false, hold2X = false, hold2Y = false, hold2RT = false;
        boolean clawOpen = true;
        double botHeading=0, clawPos = CLAW_OPEN_POSITION, MaxPwr=1;
        double LSx=0, LSy=0, RSx=0, RSy=0, rLSx=0, rLSy=0, LT=0, RT=0;
        int targetUpperArm = UA_POS_GET_SPECIMEN;
        int targetLowerArm = LA_POS_SPECIMEN;
        String tele="";

        //Initialize all hardware in hardwareMap
        setupChassis();
        setupActuators();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            tele="";
            // Set Upper/Lower Arm Positions based on state.
            switch (currentState) {
                case HOME:
                    tele += "\tHOME";
                    break;
                case GET_SPEC:
                    targetUpperArm = UA_POS_GET_SPECIMEN;
                    targetLowerArm = LA_POS_SPECIMEN;
                    tele += "\tCOLLECT SPECIMEN";
                    break;
                case GET_SAMP:
                    targetUpperArm = UA_POS_GET_SAMPLE;
                    targetLowerArm = LA_POS_SAMPLE;
                    tele += "\tCOLLECT SAMPLE";
                    break;
                case BL_HOVER:
                    targetUpperArm = UA_POS_BL_HOVER;
                    targetLowerArm = LA_POS_BL_HOVER;
                    tele += "\tHOVER OVER LOW BASKET";
                    break;
                case CL_HOVER:
                    targetUpperArm = UA_POS_CL_HOVER;
                    targetLowerArm = LA_POS_SPECIMEN;
                    tele += "\tHOVER OVER LOW CHAMBER";
                    break;
                case CL_CLIP:
                    targetUpperArm = UA_POS_CL_CLIP;
                    targetLowerArm = LA_POS_SPECIMEN;
                    tele += "\tCLIP ON LOW CHAMBER";
                    break;
                case CH_HOVER:
                    targetUpperArm = UA_POS_CH_HOVER;
                    targetLowerArm = LA_POS_SPECIMEN;
                    tele += "\tHOVER OVER HIGH CHAMBER";
                    break;
                case CH_CLIP:
                    targetUpperArm = UA_POS_CH_CLIP;
                    targetLowerArm = LA_POS_SPECIMEN;
                    tele += "\tCLIP ON HIGH CHAMBER";
                    break;
                case RL_HOVER:
                    targetUpperArm = UA_POS_RL_HOVER;
                    targetLowerArm = LA_POS_SPECIMEN;
                    tele += "\tHOVER OVER LOW RAIL";
                    break;
                case RL_LOCK:
                    targetUpperArm = UA_POS_RL_LOCK;
                    targetLowerArm = LA_POS_SPECIMEN;
                    tele += "\tLOCK ONTO LOW RAIL";
                    break;
                case MANUAL:
                    tele += "\tMANUAL";
                    break;
            }

            // Adjust currentState based on user input.
            if (gamepad2.a && !hold2A) {
                if (currentState == ArmState.GET_SPEC) { currentState = ArmState.GET_SAMP; }
                else { currentState = ArmState.GET_SPEC; }
            } else if (gamepad2.b) {
                currentState = ArmState.BL_HOVER;
            } else if (gamepad2.x && !hold2X) {
                if (currentState == ArmState.CL_HOVER) { currentState = ArmState.CL_CLIP; }
                else { currentState = ArmState.CL_HOVER; }
            } else if (gamepad2.y && !hold2Y) {
                if (currentState == ArmState.CH_HOVER) { currentState = ArmState.CH_CLIP; }
                else { currentState = ArmState.CH_HOVER; }
            } else if (gamepad1.right_bumper && !hold1RT) {
                if (currentState == ArmState.RL_HOVER) { currentState = ArmState.RL_LOCK; }
                else { currentState = ArmState.RL_HOVER; }
            } else if (gamepad1.y && !hold1Y) {
                currentState = ArmState.HOME;
            } else if (gamepad2.left_stick_y!=0){ //manual control
                currentState = ArmState.MANUAL;
                targetUpperArm += Math.round(-30*gamepad2.left_stick_y);//5000 //4650
                targetUpperArm = Math.max(UA_POS_MIN,Math.min(UA_POS_MAX,targetUpperArm));
            } else if (gamepad2.right_stick_y!=0){
                currentState = ArmState.MANUAL;
                targetLowerArm += Math.round(-3*gamepad2.right_stick_y);
                targetLowerArm = Math.max(LA_POS_MIN,Math.min(LA_POS_MAX,targetLowerArm));
            }

            // Toggle claw position when right_bumper is pressed
            if (gamepad2.right_trigger>0.1 && !hold2RT) {
                clawOpen = !clawOpen;
                clawPos = clawOpen ? CLAW_OPEN_POSITION : CLAW_CLOSED_POSITION;
                ClawServo.setPosition(clawPos);
            }

            // Control intake servo with triggers
            /* ********* SIGN MIGHT NEED TO BE REVERSED FOR INTAKE! ********* */
            if (gamepad2.left_bumper) { IntakeServo.setPower(-1.0); }
            else if (gamepad2.right_bumper) { IntakeServo.setPower(+1.0); }
            else { IntakeServo.setPower(0); }

            if (gamepad1.start) { imu.resetYaw(); }
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            hold1Y = gamepad1.y; hold1S = gamepad1.start;
            hold1RT = gamepad1.right_trigger>0.1; hold2RT = gamepad2.right_trigger>0.1;
            hold2A = gamepad2.a; hold2X = gamepad2.x; hold2Y = gamepad2.y;

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
            tele += "\n\tHeading: " + _p(botHeading);
            tele += "\n\tUpperArm: Pos: " + _p(UAMotor.getCurrentPosition()) + " / " + _p(targetUpperArm) + ", Pwr: " + _p(UAMotor.getPower());
            tele += "\n\tLowerArm: Pos: " + _p(LAMotor.getCurrentPosition()) + " / " + _p(targetLowerArm) + ", Pwr: " + _p(LAMotor.getPower());
            tele += "\n\tClaw: Pos: " + _p(ClawServo.getPosition()) + "\t" + (clawOpen ? "Open" : "Closed");
            telemetry.addData("State",tele);

            telemetry.addData("GamePad1", telemetryGP(gamepad1));
            telemetry.addData("GamePad2", telemetryGP(gamepad2));
            telemetry.update();
        }
    }

    public void setupChassis() {
        // Map the hardware motor variables.
        FLMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        BLMotor = hardwareMap.get(DcMotor.class, "backLeft");
        FRMotor = hardwareMap.get(DcMotor.class, "frontRight");
        BRMotor = hardwareMap.get(DcMotor.class, "backRight");

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
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
    }

    public void setupActuators() {
        // Map the hardware actuator variables.
        UAMotor = hardwareMap.get(DcMotor.class, "elbow");
        LAMotor = hardwareMap.get(DcMotor.class, "wrist");
        ClawServo = hardwareMap.get(Servo.class, "claw");
        IntakeServo = hardwareMap.get(CRServo.class, "intake");

        // Set Motor Direction
        UAMotor.setDirection(DcMotor.Direction.FORWARD);
        LAMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set Zero Power Behavior
        UAMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LAMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Stop and reset motor encoders
        UAMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LAMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Initialize Claw State
        //ClawOpen=false;
        //ClawChange=false;

        // Initialize Intake State
        IntakeServo.setDirection(CRServo.Direction.REVERSE);
    }

    private String telemetryGP(Gamepad gp) {
        String txt = "";
        txt += "\n\tLeftStick: \tx: " + _p(gp.left_stick_x ) + "\ty: " + _p(gp.left_stick_y ) + "\tbtn: "  + _b(gp.left_stick_button);
        txt += "\n\tRightStick:\tx: " + _p(gp.right_stick_x) + "\ty: " + _p(gp.right_stick_y) + "\tbtn: " + _b(gp.right_stick_button);
        txt += "\n\tLBump: " + _b(gp.left_bumper ) + "\tRBump: " + _b(gp.right_bumper )
                + "\tLTrig: " + _p(gp.left_trigger) + "\tRTrig: " + _p(gp.right_trigger);
        txt += "\n\tA: " + _b(gp.a) + "\tB: " + _b(gp.b) + "\tY: " + _b(gp.y) + "\tX: " + _b(gp.x)
                + "\tback: " + _b(gp.back) + "\tstart: " + _b(gp.start);
        txt += "\n\tDpad:\tL: " + _b(gp.dpad_left) + "\tR: " + _b(gp.dpad_right)
                + "\tU: " + _b(gp.dpad_up  ) + "\tD: " + _b(gp.dpad_down );
        return txt;
    }

    @SuppressLint("DefaultLocale")
    public String _p(double val) {
        return String.format("%.3f",val);
    }

    private int _b(boolean val) {
        return (val?1:0);
    }

}