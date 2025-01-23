package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "solvone4")
public class solvone4 extends LinearOpMode {
    // FIELD-CENTRIC PROGRAM
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private CRServo intake;
    private DcMotor wrist;
    private DcMotor elbow;
    private Servo claw;

    @Override
    public void runOpMode() {
        float frontLeft_power;
        float frontRight_power;
        float backLeft_power;
        float backRight_power;
        double Normal;
        String elbow_State = "";
        String wrist_State = "";
        String claw_State = "";
        int Intake_Power;
        boolean GP2a_Pressed = false;
        double clawPos; //Might need to change to float
        double rx; // Rotation offset from left to right (∆ speed to allow rotation while driving)
        double lx, ly;
        double botHeading;
        double rotX;
        double rotY;
        double denominator;
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;
        double elbowZeroPwr = 0.075;
        double elbowMaxPwr = 0.925;
        double wristZeroPwr = -0.05;
        double wristMaxPwr = 0.8;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        claw = hardwareMap.get(Servo.class, "claw");

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setPower(0);
            wrist.setPower(0);
            elbow.setPower(0);
            while (opModeIsActive()) {

                // Put run blocks here.

                // Resets "forwards" on robot
                if (gamepad1.back) {
                    imu.resetYaw();
                }

                // Gamepad 1 controls driving

                botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                lx = -gamepad1.left_stick_x;
                ly = gamepad1.left_stick_y;
                rx = -gamepad1.right_stick_x;

                // Rotate the movement direction counter to the bot's rotation
                rotX = lx * Math.cos(-botHeading) - ly * Math.sin(-botHeading);
                rotY = lx * Math.sin(-botHeading) + ly * Math.cos(-botHeading); // lx/ly/rx = joystick values, rotX/rotY = motor values

                //rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only if at least one is out of the range [-1, 1]
                denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                frontLeftPower = (rotY + rotX + rx) / denominator; // + y - (lx + rx)
                backLeftPower = (rotY - rotX + rx) / denominator; // + y + (lx - rx)
                frontRightPower = (rotY - rotX - rx) / denominator; // + y + (lx + rx)
                backRightPower = (rotY + rotX - rx) / denominator; // + y - (lx - rx)

                // This section will change it back to robot centric
                //denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1); // *∆ from separate rotY, rotX, & rx variables
                //frontLeftPower = (ly - lx - rx) / denominator; // + y - (lx + rx)
                //backLeftPower = (ly + lx - rx) / denominator; // + y + (lx - rx)
                //frontRightPower = (ly + lx + rx) / denominator; // + y + (lx + rx)
                //backRightPower = (ly - lx + rx) / denominator; // + y - (lx - rx)

                frontLeft.setPower(frontLeftPower);
                backLeft.setPower(backLeftPower);
                frontRight.setPower(frontRightPower);
                backRight.setPower(backRightPower);
                telemetry.addData("frontLeft", frontLeftPower);
                telemetry.addData("frontRight", frontRightPower);
                telemetry.addData("backLeft", backLeftPower);
                telemetry.addData("backRight", backRightPower);

                // Gamepad 2 controls extras
                // Elbow
                elbow.setPower(elbowZeroPwr - gamepad2.left_stick_y*elbowMaxPwr);
                elbow_State = ((gamepad2.left_stick_y > 0) ? "up" : ( gamepad2.left_stick_y < 0 ? "down" : "stopped"));
                /*
                if (gamepad2.left_stick_y < 0) {
                    elbow.setPower(-0.85);
                    elbow_State = "up";
                } else if (gamepad2.left_stick_y > 0) {
                    elbow.setPower(1);
                    elbow_State = "down";
                } else {
                    elbow.setPower(0.05);
                    elbow_State = "stopped";
                }
                */

                // Wrist
                wrist.setPower(wristZeroPwr - gamepad2.right_stick_y*wristMaxPwr);

                /*
                if (gamepad2.right_stick_y < 0) {
                    wrist.setPower(1);
                    wrist_State = "out";
                } else if (gamepad2.right_stick_y > 0) {
                    wrist.setPower(-0.6);
                    wrist_State = "in";
                } else {
                    wrist.setPower(-0.05);
                    wrist_State = "stopped";
                }
                */

                // Claws
                if (gamepad2.a && !GP2a_Pressed) {
                    clawPos = claw.getPosition();
                    claw.setPosition(0.9-clawPos);
                    GP2a_Pressed = true;
                    if (clawPos == 0.3) claw_State = "open";
                    else claw_State = "closed";
                } else if (!gamepad2.a && GP2a_Pressed) {
                    GP2a_Pressed = false;
                }

                // Intake
                if (gamepad2.right_bumper) {
                    intake.setPower(1);
                    Intake_Power = 1;
                } else if (gamepad2.left_bumper) {
                    intake.setPower(-1);
                    Intake_Power = -1;
                } else {
                    intake.setPower(0);
                    Intake_Power = 0;
                }

                telemetry.addData("elbow", elbow_State);
                telemetry.addData("wrist", wrist_State);
                telemetry.addData("claw", claw_State);
                telemetry.addData("Intake", Intake_Power);
                telemetry.update();
            }
        }
    }
}
