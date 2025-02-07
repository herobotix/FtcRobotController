package org.firstinspires.ftc.team22256.tele_op;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp


public class new_robot extends LinearOpMode {






    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rotator;
    private DcMotor slide;

    private Servo wrist;
    private Servo claw;

    private PIDController controller0;
    public static double p=0.05,i=0,d=0;
    public static double f = -0.05;
    double pid = 0;
    double ff = 0;
    double power = 0;
    public static double target = 0;
    public static double ticks_in_degree = 5.9744;
    public enum lift_state{
            START,
            INTAKING,
            TRANSFERRING,
            SCORING,
            RESET
    }
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        rightFront  = hardwareMap.get(DcMotor.class,"rightFront");
        leftFront  = hardwareMap.get(DcMotor.class,"leftFront");
        rightBack = hardwareMap.get(DcMotor.class,"rightBack");
        leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        rotator = hardwareMap.get(DcMotor.class,"rotator");
        slide = hardwareMap.get(DcMotor.class,"slide");

        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"claw");


        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lift_state lift_state = new_robot.lift_state.START;

        final int INTAKING_POS = 1;//These are both for the motor positions
        final int TRANSEFFERING_POS = 2;

        final double S1_OUT = 1;
        final double S2_OUT = 1;
        final double S1_IN = 2;
        final double S2_IN = 2;

        final double WRIST_TRANSFER = 1;
        final double WRIST_SCORING = 2;
        final double WRIST_IDLE = 3;

        final double CLAW_OPEN = 1;
        final double CLAW_CLOSE = 2;

        final double FLAPPER_POWER = 0.25;

        final double EXTENDED = -4100;
        final double RETRACTED = -100;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller0  = new PIDController(p,i,d); // Initialization of controller and coeffients
        pid = 0;
        ff = 0;
        power = 0;

        double changePower = 1.25;
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        target = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
           int slidePos = slide.getCurrentPosition();
       /*     switch(lift_state) {
                case START:
                    if(gamepad1.x) {
                        timer.reset();
                        S1.setPosition(S1_OUT);
                        S2.setPosition(S2_OUT);
                        if (timer.seconds() > 0.1) {
                        rotator.setTargetPosition(INTAKING_POS);
                        }
                        lift_state = new_robot.lift_state.INTAKING;
                    }
                    break;
                case INTAKING:
                    if(gamepad1.x){
                        flapper.setPower(1);
                    } else {
                        flapper.setPower(0);
                    }
                    if(gamepad1.y){
                        rotator.setTargetPosition(TRANSEFFERING_POS);
                        S1.setPosition(S1_IN);
                        S2.setPosition(S2_IN);
                        lift_state = new_robot.lift_state.TRANSFERRING;
                    }
                    break;
                case TRANSFERRING:
                    if(gamepad1.x){
                        timer.reset();
                        claw.setPosition(CLAW_OPEN);
                        wrist.setPosition(TRANSEFFERING_POS);
                        if(timer.seconds() > 500){
                            claw.setPosition(CLAW_CLOSE);
                        }
                        if(claw.getPosition() == CLAW_CLOSE){
                            target = -4100;
                            timer.reset();
                            lift_state = new_robot.lift_state.SCORING;
                        }
                    }
                    break;
                case SCORING:
                    if(Math.abs(EXTENDED - slidePos) < 5){
                        wrist.setPosition(WRIST_SCORING);
                        claw.setPosition(CLAW_OPEN);
                        if(claw.getPosition() == CLAW_OPEN){
                            wrist.setPosition(WRIST_IDLE);
                            claw.setPosition(CLAW_CLOSE);
                            target = RETRACTED;
                            rotator.setTargetPosition(INTAKING_POS);
                            S1.setPosition(S1_OUT);
                            S2.setPosition(S2_OUT);
                            lift_state = new_robot.lift_state.RESET;
                        }
                    }
                    break;
                case RESET:
                    if(Math.abs(RETRACTED - slidePos) < 5){
                        lift_state = new_robot.lift_state.INTAKING;
                    }
                    break;
                default:
                    lift_state = new_robot.lift_state.INTAKING;
            }
            if(gamepad1.start && lift_state != new_robot.lift_state.INTAKING){
                lift_state = new_robot.lift_state.INTAKING;
            }
            */
                if(gamepad1.a){
                    wrist.setPosition(0.9);
                } else if(gamepad1.b){
                    wrist.setPosition(0.35);
                }
                if(gamepad1.left_bumper){
                    claw.setPosition(1);
                }else {
                    claw.setPosition(0);
                }


            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            telemetry.addData("left stick x", gamepad1.left_stick_x);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftPower = frontLeftPower / changePower;
            backLeftPower = backLeftPower / changePower;
            frontRightPower = frontRightPower / changePower;
            backRightPower = backRightPower / changePower;

            leftFront.setPower(-frontLeftPower);
            leftBack.setPower(-backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(-backRightPower);



            controller0.setPID(p,i,d);

            pid = controller0.calculate(slidePos,target);
            ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            power = pid + ff;
            slide.setPower(power);

            if(gamepad1.dpad_up){
                target = -3850;
            } else if(gamepad1.dpad_down){
                target = -100;
            }
            if(gamepad1.right_trigger > 1){
                target = 20;
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


            telemetry.addData("target", target);
            telemetry.addData("pos", slidePos);
            telemetry.update();

        }
    }
}

