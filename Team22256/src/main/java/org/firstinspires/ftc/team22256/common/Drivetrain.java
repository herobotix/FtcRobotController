package org.firstinspires.ftc.team22256.common;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain{

    private DcMotor frontRight,backRight,frontLeft,backLeft;

    public Drivetrain(HardwareMap hardwareMap){

        frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");

    }

    public void UpdateTeleOpDrivetrain(Gamepad gamepad1){
        double changePower = 1.25;

        if (gamepad1.right_bumper) {
            changePower = 1.0; // Full speed
        } else if (gamepad1.left_bumper) {
            changePower = 1.5; // Slower
        } else {
            changePower = 1.25; // Default
        }


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        if (Math.abs(x) < 0.05) x = 0; //Deadzone to handle stick drift
        if (Math.abs(y) < 0.05) y = 0;
        if (Math.abs(rx) < 0.05) rx = 0;

        telemetry.addData("left stick x", gamepad1.left_stick_x);
        telemetry.addData("left stick y", gamepad1.left_stick_y);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1); // Power normalizer
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower = frontLeftPower / changePower;
        backLeftPower = backLeftPower / changePower;
        frontRightPower = frontRightPower / changePower;
        backRightPower = backRightPower / changePower;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }

}