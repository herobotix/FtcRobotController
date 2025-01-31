package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Calibration", group="chad")

public class Calbibration extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    Integer cpr = 27;
    double gearRatio = 15.2;
    double diameter = 2.95276;
    double cpi = (cpr * gearRatio)/ (Math.PI* diameter);
    double bias = 1;
    double driveConversion = (cpi * bias);

    public void runOpMode() {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight= hardwareMap.dcMotor.get("backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("frontLeft:", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight:", frontRight.getCurrentPosition());
        telemetry.addData("backLeft:", backLeft.getCurrentPosition());
        telemetry.addData("backRight:", backRight.getCurrentPosition());

        telemetry.update();

        waitForStart();

       moveToPosition(0.2,20);

/*
        frontLeft.setPower(0.5);
        backLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backRight.setPower(0.5);

        sleep(10000);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

 */
    }

    public void moveToPosition(double speed, double inches){

        if (inches < 5) {
            int move = (int) (Math.round(inches * driveConversion));

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
            backRight.setTargetPosition(backRight.getCurrentPosition() + move);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);

            telemetry.addLine("1");
            telemetry.addData("Speed:", speed);
            telemetry.addData("frontLeft:", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight:", frontRight.getCurrentPosition());
            telemetry.addData("backLeft:", backLeft.getCurrentPosition());
            telemetry.addData("backLeft POWER:", backLeft.getPower());
            telemetry.addData("backRight:", backRight.getCurrentPosition());

            telemetry.update();

            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                telemetry.addLine("1");
                telemetry.addData("Speed:", speed);
                telemetry.addData("frontLeft:", frontLeft.getCurrentPosition());
                telemetry.addData("frontRight:", frontRight.getCurrentPosition());
                telemetry.addData("backLeft:", backLeft.getCurrentPosition());
                telemetry.addData("backLeft POWER:", backLeft.getPower());
                telemetry.addData("backRight:", backRight.getCurrentPosition());

                telemetry.update();
            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            telemetry.addLine("1");
            telemetry.addData("Speed:", speed);
            telemetry.addData("frontLeft:", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight:", frontRight.getCurrentPosition());
            telemetry.addData("backLeft:", backLeft.getCurrentPosition());
            telemetry.addData("backRight:", backRight.getCurrentPosition());

            telemetry.update();




        } else {
            int move1 = (int) (Math.round ((inches - 5) * driveConversion));
            int moveFL = frontLeft.getCurrentPosition() + (int) (Math.round(inches * driveConversion));
            int moveFR = frontRight.getCurrentPosition() + (int) (Math.round(inches * driveConversion));
            int moveBL = backLeft.getCurrentPosition() + (int) (Math.round(inches * driveConversion));
            int moveBR = backRight.getCurrentPosition() + (int) (Math.round(inches * driveConversion));

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move1);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + move1);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + move1);
            backRight.setTargetPosition(backRight.getCurrentPosition() + move1);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);

            telemetry.addLine("2");
            telemetry.addData("Speed:", speed);
            telemetry.addData("frontLeft:", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight:", frontRight.getCurrentPosition());
            telemetry.addData("backLeft:", backLeft.getCurrentPosition());
            telemetry.addData("backLeft POWER:", backLeft.getPower());
            telemetry.addData("backRight:", backRight.getCurrentPosition());

            telemetry.update();

            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                telemetry.addLine("2");
                telemetry.addData("Speed:", speed);
                telemetry.addData("frontLeft:", frontLeft.getCurrentPosition());
                telemetry.addData("frontRight:", frontRight.getCurrentPosition());
                telemetry.addData("backLeft:", backLeft.getCurrentPosition());
                telemetry.addData("backLeft POWER:", backLeft.getPower());
                telemetry.addData("backRight:", backRight.getCurrentPosition());

                telemetry.update();
            }
            frontLeft.setTargetPosition(moveFL);
            frontRight.setTargetPosition(moveFR);
            backLeft.setTargetPosition(moveBL);
            backRight.setTargetPosition(moveBR);

            frontLeft.setPower(0.1);
            frontRight.setPower(0.1);
            backLeft.setPower(0.1);
            backRight.setPower(0.1);
            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {

                telemetry.addLine("2");
                telemetry.addData("Speed:", speed);
                telemetry.addData("frontLeft:", frontLeft.getCurrentPosition());
                telemetry.addData("frontRight:", frontRight.getCurrentPosition());
                telemetry.addData("backLeft:", backLeft.getCurrentPosition());
                telemetry.addData("backLeft POWER:", backLeft.getPower());
                telemetry.addData("backRight:", backRight.getCurrentPosition());

                telemetry.update();
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            telemetry.addLine("2");
            telemetry.addData("Speed:", speed);
            telemetry.addData("frontLeft:", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight:", frontRight.getCurrentPosition());
            telemetry.addData("backLeft:", backLeft.getCurrentPosition());
            telemetry.addData("backLeft POWER:", backLeft.getPower());
            telemetry.addData("backRight:", backRight.getCurrentPosition());

            telemetry.update();
        }
    }
}
