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
    private Servo intake;
    private CRServo intake_2;

    private PIDController controller0;
    final public  static double p=0.05,i=0,d=0;
    final public static double f = -0.05;
    double pid = 0;
    double ff = 0;
    double power = 0;
    public static double target = 0;
    public static double ticks_in_degree = 5.9744;
    public enum Intake{
    INTAKING,
        OUTAKING,
        NEUTRAL
    }
    Intake intake_1 = Intake.NEUTRAL;
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
        intake = hardwareMap.get(Servo.class,"intake");
        intake_2 = hardwareMap.get(CRServo.class,"intake_2");


        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller0  = new PIDController(p,i,d); // Initialization of controller and coeffients
        pid = 0;
        ff = 0;
        power = 0;

        double changePower = 1.25;
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        boolean BP = false;


        waitForStart();
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        target = 0;
        int times_pressed = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
           int slidePos = slide.getCurrentPosition();

                if(gamepad1.right_trigger > 0.1){
                    intake_2.setPower(1);
                } else if(gamepad1.right_bumper){
                    intake_2.setPower(-1);
                } else {
                    intake_2.setPower(0);
                }

                if(gamepad1.dpad_right){
                    target = -3600;

                }

/*
            switch(intake_1){
                case NEUTRAL:
                    if(gamepad1.b){
                        intake_1 = Intake.INTAKING;
                    } else if(gamepad1.y){
                        intake_1 = Intake.OUTAKING;
                    }
                    break;
                case INTAKING:
                    timer.reset();
                    if (!(timer.seconds()  > 1.5)){
                        intake_2.setPower(-1);
                }
                    intake_2.setPower(0);
                    intake_1 = Intake.NEUTRAL;
                    break;
                case OUTAKING:
                    timer.reset();
                    if (!(timer.seconds()  > 1.5)){
                        intake_2.setPower(1);
                    }
                    intake_2.setPower(0);
                    intake_1 = Intake.NEUTRAL;
                    break;
            }
*/


            if (gamepad1.a && !BP) {
                times_pressed++;
                BP = true;
            } else if (!gamepad1.a && BP) {
                BP = false;
            }
            if(times_pressed % 2 == 0){
                wrist.setPosition(0.1);
                intake.setPosition(1);
            } else {
                wrist.setPosition(1);
                intake.setPosition(0);
            }
            double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad2.right_stick_x;

            telemetry.addData("left stick x", gamepad2.left_stick_x);
            telemetry.addData("left stick y", gamepad2.left_stick_y);
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


            if(target < -3850){
                target = -3850;
            } else if(target > -10){
                target = -20;
            }
            //Hi Armani if you keep tis in this is where the self destruct button would go
            boolean self_destruct = false;


            target = target + (gamepad1.left_stick_y * 2.5);

            if(gamepad1.dpad_up){
                target = -3850;
                times_pressed = 0;
            } else if(gamepad1.dpad_down){
                target = -35;
            }
            if(gamepad1.back){
                target = 20;
                slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


            telemetry.addData("target", target);
            telemetry.addData("pos", slidePos);
            telemetry.update();
            telemetry.addData("times pressed",times_pressed);
        }
    }
}

