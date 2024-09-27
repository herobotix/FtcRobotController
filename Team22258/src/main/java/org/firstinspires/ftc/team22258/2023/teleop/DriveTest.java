package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

public class DriveTest extends LinearOpMode {
    Robot robot;
    private PIDController controller;
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

        robot.armRight.setDirection(DcMotorEx.Direction.REVERSE);
        robot.armRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.armLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.armLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.armRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        controller = new PIDController(p, i, d);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad2.dpad_up) {
                target = target + 1;
            } else if (gamepad2.dpad_down) {
                target = target - 1;
            } else if (gamepad2.dpad_left) {
                target = 430;
            } else if (gamepad2.dpad_right){
                target = 100;
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
            telemetry.update();
        }
    }
}