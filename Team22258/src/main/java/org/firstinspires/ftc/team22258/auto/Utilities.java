package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Utilities {

    /*

    double t, minVel, maxVel, fLa, fRa, bLa, bRa;
    int n;

    public void findMaxVel() {

        // INSERT INTO START

        t = runtime.seconds();
        n = 1;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setVelocity(MAX_LINVEL);
        frontRight.setVelocity(MAX_LINVEL);
        backLeft.setVelocity(MAX_LINVEL);
        backRight.setVelocity(MAX_LINVEL);

        // INSERT INTO START LOOP

        if (runtime.seconds() - t > 2.0) {
            if (gamepad1.b) requestOpModeStop();
            frontLeft.setVelocity(0);
            frontRight.setVelocity(0);
            backLeft.setVelocity(0);
            backRight.setVelocity(0);
        } else {
            double fL, fR, bL, bR;
            fL = frontLeft.getVelocity();
            fR = frontRight.getVelocity();
            bL = backLeft.getVelocity();
            bR = backRight.getVelocity();

            maxVel = Math.max(maxVel, fL);
            maxVel = Math.max(maxVel, fR);
            maxVel = Math.max(maxVel, bL);
            maxVel = Math.max(maxVel, bR);

            if (maxVel > 2200) {
                minVel = Math.min(minVel, fL);
                minVel = Math.min(minVel, fR);
                minVel = Math.min(minVel, bL);
                minVel = Math.min(minVel, bR);

                fLa = (fLa * n + fL) / (n + 1);
                fRa = (fRa * n + fR) / (n + 1);
                bLa = (bLa * n + bL) / (n + 1);
                bRa = (bRa * n + bR) / (n + 1);
                n++;

            } else minVel = fLa = fRa = bLa = bRa = 2200;

            telemetry.addData("fLvel: ", fL);
            telemetry.addData("fRvel: ", fR);
            telemetry.addData("bLvel: ", bL);
            telemetry.addData("bRvel: ", bR);
            telemetry.addData("minVel: ", minVel);
            telemetry.addData("maxVel: ", maxVel);
            telemetry.addData("fLavg: ", fLa);
            telemetry.addData("fRavg: ", fRa);
            telemetry.addData("bLavg: ", bLa);
            telemetry.addData("bRavg: ", bRa);
            telemetry.update();
        }


    }
     */

    /*
    public void imuInit() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }
    */

    /*
    public void imuTelemetry() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        angPos = orientation.getYaw(AngleUnit.DEGREES);
        angVel = angularVelocity.zRotationRate;

        telemetry.addData("Angular Position", "%.2f Deg. (Heading)", angPos);
        //telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        //telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Angular Velocity", "%.2f Deg/Sec", angVel);
        //telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        //telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
    }
    */

    /*
    public void move(double distance, double angle, double percVel) {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double conv, distL, distR, velL, velR;
        conv = 112/(7.5*Math.PI);	// ticks per centimeter
        distL = conv * (distance + 10.0*angle*(Math.PI/180.0));	// Distance change with angle change
        distR = conv * (distance - 10.0*angle*(Math.PI/180.0));	// +/- angle turns right/left
        velL = distL/time;
        velR = distR/time;

        //telemetry.addData("distL theory:", distL);
        //telemetry.addData("distL actual:", 0.5*(FrontLeft.getCurrentPosition()+BackLeft.getCurrentPosition()));
        //telemetry.addData("distR theory", distR);
        //telemetry.addData("distR actual:", 0.5*(FrontRight.getCurrentPosition()+BackRight.getCurrentPosition()));
        //telemetry.addData("velL theory", velL);
        //telemetry.addData("velL actual", 0.5*(FrontLeft.getVelocity()+BackLeft.getVelocity()));
        //telemetry.addData("velR theory", velR);
        //telemetry.addData("velR actual", 0.5*(FrontRight.getVelocity()+BackRight.getVelocity()));

        FrontLeft.setTargetPosition((int) distL);
        FrontRight.setTargetPosition((int) distR);
        BackLeft.setTargetPosition((int) distL);
        BackRight.setTargetPosition((int) distR);

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setVelocity(velL);
        FrontRight.setVelocity(velR);
        BackLeft.setVelocity(velL);
        BackRight.setVelocity(velR);

    }
     */

    /*
    public void turnImu(double dAngPos) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        angPos = orientation.getYaw(AngleUnit.DEGREES);
        angVel = angularVelocity.zRotationRate;

        double trgAngle = angPos - dAngPos;
        double maxPwr = Math.min(Math.abs(angPos),90)/90;
        double curPwr = 0, trgPwr = maxPwr;
        int rampCycles = 10;
        double stpPwr = (trgPwr - curPwr) / rampCycles;

        while(Math.signum(dAngPos)*(trgAngle-angPos)<0) {
            //curPwr += stpPwr;
            //if ()

            curPwr = 0.2*Math.signum(dAngPos);
            frontLeft.setPower(curPwr);
            backLeft.setPower(curPwr);
            frontRight.setPower(-curPwr);
            backRight.setPower(-curPwr);

            angPos = orientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("Yaw Value", angPos);
            telemetry.update();
        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
    */

    /*
    public void turnImu2(double dAngPos) {
        double dtoi = 23.50/90.0;
        int move = (int)(Math.round(dAngPos * 1.1 * dtoi * INCHES_TO_TICKS));  // 1.1 for overshoot

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        angPos = orientation.getYaw(AngleUnit.DEGREES);
        angVel = angularVelocity.zRotationRate;

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() - move);

        //frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double old = angPos, trg = angPos - dAngPos;
        double trgUP = angPos - 0.1*dAngPos, trgDOWN = angPos - 0.9*dAngPos;
        double maxPwr = Math.min(Math.abs(dAngPos),90)/90;
        double curPwr = 0.1*maxPwr, trgPwr = maxPwr;
        //double stpPwr = (trgPwr - curPwr) / rampCycles;

        while(Math.signum(dAngPos)*(trg-angPos)<0) {
            frontLeft.setPower(curPwr);
            backLeft.setPower(curPwr);
            frontRight.setPower(-curPwr);
            backRight.setPower(-curPwr);

            angPos = orientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("Yaw Value", angPos);
            telemetry.update();

            curPwr =

            //curPwr += stpPwr;
            //if ()

            curPwr = 0.2*Math.signum(dAngPos);

        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }
    */

}
