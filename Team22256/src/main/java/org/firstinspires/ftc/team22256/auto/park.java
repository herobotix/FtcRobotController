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
import org.firstinspires.ftc.team22256.methods.robot;
@Config
@Autonomous


public class park extends LinearOpMode {

    robot robot = new robot(hardwareMap);

    private PIDController controller0;
    public static double p=0.05,i=0,d=0;
    public static double f = -0.05;
    double pid = 0;
    double ff = 0;
    double power = 0;

    public enum lift_state{
        START,
        INTAKING,
        TRANSFERRING,
        SCORING,
        RESET
    }
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
    public enum auto{
        STRAFE,
        FORWARD,
        TOUCH
    }
    auto Auto   = auto.STRAFE  ;
    public static double ticks_in_degree = 5.9744;
    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller0  = new PIDController(p,i,d); // Initialization of controller and coeffients

        pid = 0;
        ff = 0;
        power = 0;



        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        robot.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.rightBack.setTargetPosition(0);
        robot.leftBack.setTargetPosition(0);
        robot.leftFront.setTargetPosition(0);
        robot.rightFront.setTargetPosition(0);

        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.rightBack.setPower(1);
        robot.leftBack.setPower(1);
        robot.leftFront.setPower(1);
        robot.rightFront.setPower(1);

        double target = 0;
        while (opModeIsActive()) {

        switch (Auto){
            case STRAFE:
                robot.strafeToPosition(0.75,72);
                Auto = auto.FORWARD;
                break;
            case FORWARD:
                target = -400;
                Auto = auto.TOUCH;
                break;
            case TOUCH:
                robot.driveToPosition(20,0.25);
        }

            controller0.setPID(p,i,d);
            int slidePos = robot.slide.getCurrentPosition();
            pid = controller0.calculate(slidePos,target);
            ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            power = pid + ff;
            robot.slide.setPower(power);








            telemetry.update();
        }
    }
}