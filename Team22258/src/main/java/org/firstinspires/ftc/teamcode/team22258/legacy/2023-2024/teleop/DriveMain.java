package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class DriveMain extends LinearOpMode {
    Robot robot;
    public PIDController controller;
    public static double p = 0.004, i = 0, d = 0;
    public static double f = 0.15;
    public static int target = 0;
    public final double motorRatio = 27;
    public final double gearRatio = 54.8;
    public final double gearFinal = motorRatio * gearRatio;
    public final double ticksInDegreesArm = gearFinal / 180.0; //54.8 * 27 gear ratio
    int leftPos;
    int rightPos;
    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        int arm_target_position = 0;
        double changePower = 1.25;

        robot.armRight.setDirection(DcMotorEx.Direction.REVERSE);
        robot.armRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.armLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.armLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.armRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(p, i, d);

        waitForStart();

        robot.wristRight.setPosition(0.85);
        robot.wristLeft.setPosition(0.45);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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

            robot.frontLeft.setPower(frontLeftPower);
            robot.backLeft.setPower(backLeftPower);
            robot.frontRight.setPower(frontRightPower);
            robot.backRight.setPower(backRightPower);

            if (gamepad1.left_bumper) { //slow
                changePower = 2;
            }
            if (gamepad1.right_bumper) { //fast
                changePower = 1.25;
            }
            if (gamepad1.a) {
                robot.launcher.setPosition(1);
            }
            if (gamepad1.b) {
                robot.launcher.setPosition(0);
            }

            if (gamepad2.a){
                robot.wristArm.setPosition(0);
            }
            if (gamepad2.b) {
                robot.wristArm.setPosition(1);
            }

            if(gamepad2.x) { //reset ARM encoders
                robot.armRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                robot.armLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                robot.armLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                robot.armRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad2.left_bumper) {
                robot.wristLeft.setPosition(0.8);
            } else {
                robot.wristLeft.setPosition(0.45);
            }
            if (gamepad2.right_bumper) {
                robot.wristRight.setPosition(0.5);
            } else {
                robot.wristRight.setPosition(0.85);
            }

            if(gamepad2.right_trigger > 0) {
                robot.armMiddle.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0) {
                robot.armMiddle.setPower(-gamepad2.left_trigger);
            }

            //PID for ARM
            if(gamepad2.dpad_up) {
                target = target + 1;
            } else if (gamepad2.dpad_down) {
                target = target - 1;
            } else if (gamepad2.dpad_left) { //backboard
                target = 400;
            } else if (gamepad2.dpad_right) { //pixel pickup
                target = 100;
            }
            if (gamepad2.y) { //straight up
                target = 300;
            }

            controller.setPID(p, i, d);
            leftPos = robot.armLeft.getCurrentPosition();
            rightPos = robot.armRight.getCurrentPosition();
            double leftPid = controller.calculate(leftPos, target);
            double rightPid = controller.calculate(rightPos, target);
            double ff = Math.cos(Math.toRadians((target / ticksInDegreesArm))) * f;

            double leftPower = (leftPid + ff) * 0.5;
            double rightPower = (rightPid+ ff) * 0.5;

            robot.armLeft.setPower(leftPower);
            robot.armRight.setPower(rightPower);

            telemetry.addData("left pos", leftPos);
            telemetry.addData("right pos", rightPos);
            telemetry.addData("target", target);
            telemetry.addData("Speed", changePower);
            telemetry.addData("Front left", frontLeftPower);
            telemetry.addData("Front right", frontRightPower);
            telemetry.addData("Back left", backLeftPower);
            telemetry.addData("Arm Target Position:", arm_target_position);
            telemetry.addData("Arm Position: ", robot.armLeft.getCurrentPosition());
            telemetry.addData("arm right", robot.armLeft.getPower());
            telemetry.addData("arm left", robot.armRight.getPower());
            telemetry.addData("arm extension", robot.armMiddle.getPower());

            telemetry.update();
        }
    }
}