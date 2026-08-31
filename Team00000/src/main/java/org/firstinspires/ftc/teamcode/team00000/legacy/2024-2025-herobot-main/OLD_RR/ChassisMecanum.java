package org.firstinspires.ftc.team00000;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.HashMap;
import java.util.Map;

public class ChassisMecanum {

    public DcMotorEx motorFL, motorFR, motorBL, motorBR;

    double msCPR = 28;  /*motor shaft - counts per revolution*/
    double gearRatio = 15.2;
    double osCPR = msCPR * gearRatio;  /*output shaft - counts per revolution*/
    double diameter = 2.95276;  /*diameter in inches*/
    double bias = 1;
    double CPI = bias * osCPR / (Math.PI * diameter);  /*counts per inch*/
    IMU imu;
    Orientation angles;

    double currentX=0, currentY=0, currentA=0;
    double targetX=0, targetY=0, targetA=0;

    double integralSumFL = 0;
    double integralSumFR = 0;
    double integralSumBL = 0;
    double integralSumBR = 0;
    private double lastErrorFL = 0;
    private double lastErrorFR = 0;
    private double lastErrorBL = 0;
    private double lastErrorBR = 0;
    double Kp = 1;
    double Ki = 0;
    double Kd = 0;
    ElapsedTime timer = new ElapsedTime();

    public ChassisMecanum(HardwareMap hardwareMap, boolean fieldCentric) {

        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize( new IMU.Parameters( new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
    }

    public void moveToPositionAngleFC(double tX, double tY, double tA) {
        if (tX != targetX || tY != targetY || tA != targetA) {
            // reset Targets!
        }
        double dX = targetX - currentX, dY = targetY - currentY;
        double dA1 = Math.atan2(dY,dX);
        dA1 -= 2*Math.PI*Math.floor((dA1/(2*Math.PI)) - 0.5);     // ensure dA1 in range (-π,π]
        double dA2 = targetA - (currentA + dA1);
        dA2 -= 2*Math.PI*Math.floor((dA2/(2*Math.PI)) - 0.5);     // ensure dA2 in range (-π,π]


        double cntFL = CPI * 0;

        motorFL.setTargetPosition(0);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*
    public void setPIDFPosition(Map<String, Integer> target) {
        double currentTime = timer.seconds();

        double errorFL = target.get("motorFL") - motorFL.getCurrentPosition();
        double errorFR = target.get("motorFR") - motorFR.getCurrentPosition();
        double errorBL = target.get("motorBL") - motorBL.getCurrentPosition();
        double errorBR = target.get("motorBR") - motorBR.getCurrentPosition();

        integralSumFL += errorFL * currentTime;
        integralSumFR += errorFR * currentTime;
        integralSumBL += errorBL * currentTime;
        integralSumBR += errorBR * currentTime;

        double derivativeFL = (errorFL - lastErrorFL) / currentTime;
        double derivativeFR = (errorFR - lastErrorFR) / currentTime;
        double derivativeBL = (errorBL - lastErrorBL) / currentTime;
        double derivativeBR = (errorBR - lastErrorBR) / currentTime;

        lastErrorFL = errorFL;
        lastErrorFR = errorFR;
        lastErrorBL = errorBL;
        lastErrorBR = errorBR;

        motorFL.setPower((errorFL * Kp) + (derivativeFL * Kd) + (integralSumFL * Ki));
        motorFR.setPower((errorFR * Kp) + (derivativeFR * Kd) + (integralSumFR * Ki));
        motorBL.setPower((errorBL * Kp) + (derivativeBL * Kd) + (integralSumBL * Ki));
        motorBR.setPower((errorBR * Kp) + (derivativeBR * Kd) + (integralSumBR * Ki));

        timer.reset();
    }
    */


}

