package org.firstinspires.ftc.schaub.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoDrive {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    public AutoDrive(DcMotor i_leftFront,
                     DcMotor i_leftBack,
                     DcMotor i_rightFront,
                     DcMotor i_rightBack)
    {
        leftFrontDrive = i_leftFront;
        leftBackDrive = i_leftBack;
        rightFrontDrive = i_rightFront;
        rightBackDrive = i_rightBack;
    }

    public void run(VisionTarget target, Telemetry telemetry)
    {
        turnTo(target.getHeading(), 2.5, telemetry);
//
//        // Convert target heading to radians for x, y movement calculations
//        double radians = Math.toRadians(target.getHeading()); // Target heading in radians
//        double x = Math.cos(radians) * target.getDistance();  // X component of movement (forward/backward, strafing)
//        double y = Math.sin(radians) * target.getDistance();  // Y component of movement (left/right strafing)
//
//        // Calculate the rotational power to face the target
//        // The magnitude of the rotational power can be scaled (e.g., by 0.01) to make the turn smooth.
//        double rotation = target.getHeading() * 0.01;
//
//        // Power calculations for each motor
//        double leftFrontPower = y + x + rotation;
//        double rightFrontPower = y - x - rotation;
//        double leftBackPower = y - x + rotation;
//        double rightBackPower = y + x - rotation;
//
//        // Normalize power to ensure values are within -1.0 to 1.0
//        double maxPower = Math.max(1.0, Math.abs(leftFrontPower));
//        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
//        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
//        maxPower = Math.max(maxPower, Math.abs(rightBackPower));
//
//        leftFrontPower /= maxPower;
//        rightFrontPower /= maxPower;
//        leftBackPower /= maxPower;
//        rightBackPower /= maxPower;
//
//        // Set power to the motors
//        leftFrontDrive.setPower(leftFrontPower / 5.0);
//        rightFrontDrive.setPower(rightFrontPower / 5.0);
//        leftBackDrive.setPower(leftBackPower / 5.0);
//        rightBackDrive.setPower(rightBackPower / 5.0);
//
//        // Show the wheel power.
//        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
    }

    public void turnTo(double bearing, double tolerance, Telemetry telemetry) {

        // Proportional gain factor for controlling how fast the robot turns
        double kP = 0.1; // You can tune this value based on your robot's response

        // Keep turning until the robot is within the tolerance of the target heading
        if (Math.abs(bearing) > tolerance) {
            // Calculate the turn power based on the heading difference
            double turnPower = bearing * kP;

            // Cap the power to [-1.0, 1.0] range
            turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

            // Set motor powers for turning in place
            leftFrontDrive.setPower(turnPower);
            rightFrontDrive.setPower(-turnPower);
            leftBackDrive.setPower(turnPower);
            rightBackDrive.setPower(-turnPower);

            // Telemetry for debugging
            telemetry.addData("Target Bearing", bearing);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }
        else
        {
            leftFrontDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
        }
    }

}
