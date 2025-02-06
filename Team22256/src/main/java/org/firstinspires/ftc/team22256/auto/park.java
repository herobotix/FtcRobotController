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

    public static double ticks_in_degree = 5.9744;

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


        robot.leftFront.setTargetPosition(0);
        robot.rightFront.setTargetPosition(0);
        robot.leftBack.setTargetPosition(0);
        robot.rightBack.setTargetPosition(0);


        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFront.setVelocity(1500);
        robot.rightFront.setVelocity(1500);
        robot.leftBack.setVelocity(1500);
        robot.rightBack.setVelocity(1500);

        while (opModeIsActive()) {

            ;
            if (opModeIsActive()) {
                robot.strafeToPosition(1500, 50);


                telemetry.addData("power", robot.leftFront.getPower());
                telemetry.addData("power2", robot.leftBack.getPower());

            }
        }
    }
}