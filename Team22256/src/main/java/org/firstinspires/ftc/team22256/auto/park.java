package org.firstinspires.ftc.team22256.auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team22256.methods.Robot;

@Config
@Autonomous


public class park extends LinearOpMode {

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
        SIX
    }
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller0 = new PIDController(p, i, d); // Initialization of controller and coeffients
        state State = state.ONE;
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


            if (opModeIsActive()) {
                switch (State) {
                    case ONE:
                        robot.score_pos();
                        robot.strafeToPosition(1000, -24);
                        robot.driveToPosition(-24, 1000);
                        State = state.TWO;
                        break;
                    case TWO:
                        robot.turn(51, 700);
                        target = (-3700);

                        State = state.THREE;
                        break;
                    case THREE:
                        sleep(1000);
                        robot.driveToPosition(-9, 500);
                        State = state.FOUR;
                        break;
                    case FOUR:
                        robot.intake(3);
                        State = state.FIVE;
                        break;
                    case FIVE:
                        timer.reset();
                        if(timer.seconds() >3) {
                            robot.driveToPosition(20, 1500);
                            State = state.SIX;
                        }
                        break;
                    case SIX:
                       timer.reset();
                        if(timer.seconds() > 7) {
                            target = (-170);
                        }
                        break;
                }

                    controller0.setPID(p, i, d);
                    int slidePos = robot.slide.getCurrentPosition();
                    pid = controller0.calculate(slidePos, target);
                    ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
                    power = pid + ff;
                    robot.slide.setPower(power);



            }
        }
    }
}