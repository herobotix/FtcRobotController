package org.firstinspires.ftc.team22256.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team22256.methods.Robot;

@Config
@Autonomous


public class sample extends LinearOpMode {

    Robot robot;

    private PIDController controller0;
    public static double p = 0.05, i = 0, d = 0;
    public static double f = -0.05;
    double pid = 0;
    double ff = 0;
    double power = 0;
    double target;
    public static double ticks_in_degree = 5.9744;
    public enum state{
        ONE,
        TWO,
        THREE,
        FOUR,
        FIVE,
        SIX,
        SEVEN,
        EIGHT,
        NINE,
        FINISHED
    }
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller0 = new PIDController(p, i, d); // Initialization of controller and coeffients

        pid = 0;
        ff = 0;
        power = 0;

        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        state State = state.ONE;
        robot.leftFront.setTargetPosition(0);
        robot.rightFront.setTargetPosition(0);
        robot.leftBack.setTargetPosition(0);
        robot.rightBack.setTargetPosition(0);


        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFront.setVelocity(0);
        robot.rightFront.setVelocity(0);
        robot.leftBack.setVelocity(0);
        robot.rightBack.setVelocity(0);

        while (opModeIsActive()) {




            switch (State) {
                case ONE:
                    //Beginning of first sample
                    robot.score_pos();
                    robot.strafeToPosition(1000, -24);
                    robot.driveToPosition(-20, 1000);
                    State = state.TWO;
                    break;
                case TWO:
                    robot.turn(51, 700);
                    robot.setTarget(-3800,true);
                    State = state.THREE;
                    break;
                case THREE:
                    robot.driveToPosition(-13, 700);
                    State = state.FOUR;
                    break;
                case FOUR:
                    robot.intake(0.75);
                    State = state.FIVE;
                    break;
                case FIVE:
                    robot.driveToPosition(18, 1300);
                    robot.setTarget(0,false);
                    State = state.SIX;
                    break;
                case SIX:
                    //Beginning of second sample
                    robot.turn(51,800);//fixed
                    robot.driveToPosition(-2.5,600);
                    robot.intake_pos();
                    sleep(1000);
                    robot.strafe_intake(-18,600);
                    State = state.SEVEN;
                    break;
                case SEVEN:
                    robot.driveToPosition(-10,500);
                    robot.turn(-51,600);
                   // robot.strafeToPosition(500,7);
                    robot.score_pos();
                    robot.setTarget(-3850,true);
                    State = state.EIGHT;
                    break;
                case EIGHT:
                    robot.driveToPosition(-5,700);
                    robot.intake(3);
                    State = state.NINE;
                    break;
                case NINE:
                    robot.driveToPosition(15,1000);
                    robot.setTarget(0,false);
                    State = state.FINISHED;
                    break;
                case FINISHED:
                    break;
                default:
                    State = state.FINISHED;
                    break;
            }




        }
    }
}
