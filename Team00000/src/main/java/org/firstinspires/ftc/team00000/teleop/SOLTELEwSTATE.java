package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "SOLTELEwSTATE")
public class SOLTELEwSTATE extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private CRServo intake;
    private DcMotor wrist;
    private DcMotor elbow;
    private Servo claw;
    private IMU imu;

    @Override
    public void runOpMode() {
        float frontLeft_power;
        float frontRight_power;
        float backLeft_power;
        float backRight_power;
        double Normal;
        String elbow_State;
        String wrist_State;
        String claw_State;
        int Intake_Power;
        boolean GP2a_Pressed = false;
        double clawPos; //Might need to change to float
        double rx = gamepad1.right_stick_x; // Rotation offset from left to right (changes speed to allow rotation while driving)

        frontLeft = hardwareMap.get(DcMotor.class, "FLMotor");
        backLeft = hardwareMap.get(DcMotor.class, "BLMotor");
        frontRight = hardwareMap.get(DcMotor.class, "FRMotor");
        backRight = hardwareMap.get(DcMotor.class, "BRMotor");
        intake = hardwareMap.get(CRServo.class, "IntakeServo");
        wrist = hardwareMap.get(DcMotor.class, "WristMotor");
        elbow = hardwareMap.get(DcMotor.class, "ArmMotor");
        claw = hardwareMap.get(Servo.class, "ClawServo");

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setPower(0);
            wrist.setPower(0);
            elbow.setPower(0);
            while (opModeIsActive()) {

                // Put run blocks here.

                // Resets "forwards" on robot
                if (gamepad1.options) {
                    imu.resetYaw();
                    telemetry.addData("imu", "yaw reset"); // *Bot code doesn't like this line, everything else is fine
                }

                // Gamepad 1 controls driving

                // gm0 driving
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                // Bot doesn't recognise x and y variables
                double rotX = -gamepad1.left_stick_x * Math.cos(-botHeading) - gamepad1.left_stick_y * Math.sin(-botHeading); // *Doesn't recognise x and y variables
                double rotY = -gamepad1.left_stick_x * Math.sin(-botHeading) + gamepad1.left_stick_y * Math.cos(-botHeading); // x/y = joystick values, rotX/rotY = motor values

                rotX = rotX * 1.1;  // Counteract imperfect strafing

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only if at least one is out of the range [-1, 1]
                // *x-axis flipped, fix later
                double denominator = Math.max(Math.abs(rotY + rotX + rx), 1); // *∆ from separate rotY, rotX, & rx variables
                double frontLeftPower = (rotY + rotX + rx) / denominator; // +y - (lx + rx)
                double backLeftPower = (rotY - rotX + rx) / denominator; // +y + (lx - rx)
                double frontRightPower = (rotY - rotX - rx) / denominator; // +y + (lx + rx)
                double backRightPower = (rotY + rotX - rx) / denominator; // +y - (lx - rx)

                frontLeft.setPower(frontLeftPower);
                backLeft.setPower(backLeftPower);
                frontRight.setPower(frontRightPower);
                backRight.setPower(backRightPower);
                telemetry.addData("frontLeft", frontLeftPower);
                telemetry.addData("frontRight", frontRightPower);
                telemetry.addData("backLeft", backLeftPower);
                telemetry.addData("backRight", backRightPower);

                telemetry.update();
            }
        }
    }
}
