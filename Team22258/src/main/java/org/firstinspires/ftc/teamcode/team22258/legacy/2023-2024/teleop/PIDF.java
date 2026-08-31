package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PIDF extends OpMode {
    private PIDController controller;
    private static double p1 = 0, i1 = 0, d1 = 0;
    private static  double f1 = 0;
    private static int target1 = 0;

    private final double motorRatio1 = 27;
    private final double gearRatio1 = 54.8;
    private final double gearFinal1 = motorRatio1 * gearRatio1;
    private final double ticksInDegreesArm1 = gearFinal1 / 180.0; //54.8 * 27 gear ratio

    private DcMotorEx armLeft;
    private DcMotorEx armRight;

    @Override
    public void init() {
        controller = new PIDController( p1, i1, d1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armRight = hardwareMap.get(DcMotorEx.class, "armRight");
        armLeft = hardwareMap.get(DcMotorEx.class, "armLeft");

        armRight.setDirection(DcMotorEx.Direction.REVERSE);

    }

    @Override
    public void loop() {
        controller.setPID(p1, i1, d1);
        int leftPos = armLeft.getCurrentPosition();
        int rightPos = armRight.getCurrentPosition();
        double leftPid = controller.calculate(leftPos, target1);
        double rightPid = controller.calculate(rightPos, target1);
        double ff = Math.cos(Math.toRadians((target1 / ticksInDegreesArm1))) * f1;

        double leftPower = (leftPid + ff) * 0.5;
        double rightPower = (rightPid+ ff) * 0.5;
        armLeft.setPower(leftPower);
        armRight.setPower(rightPower);
        telemetry.addData("left pos", leftPos);
        telemetry.addData("right pos", rightPos);
        telemetry.addData("target", target1);
        telemetry.update();
    }
}
