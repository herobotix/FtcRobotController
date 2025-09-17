package org.firstinspires.ftc.team22257.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleop1")
public class teleop1 extends LinearOpMode {
    // FIELD-CENTRIC PROGRAM
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor LnM; //Launcher Motor
    private CRServo AgtS; //Agitator Servo
    enum state{
        UP,
        RESET;

    }
    @Override
    public void runOpMode() {
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


        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        LnM = hardwareMap.get(DcMotor.class, "Launcher Motor");
        AgtS = hardwareMap.get(CRServo.class, "Agitator Servo");
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

            while (opModeIsActive()) {

                // Resets "forwards" on robot
                if (gamepad1.back) {
                    imu.resetYaw();
                }

                // Gamepad 1 controls driving


                if (gamepad1.start) imu.resetYaw();
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
                telemetry.addData("heading", botHeading);
                telemetry.update();
            }
        }
    }
}
