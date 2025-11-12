
package org.firstinspires.ftc.team22256.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp

public class temp2 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor backLeft,backRight,frontLeft,frontRight;
    private DcMotorEx shooterLeft, shooterRight;
    private DcMotor intake;
    private DcMotor turret;

    private Servo kicker;

    @Override
    public void runOpMode() {


        frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");

        turret  = hardwareMap.get(DcMotor.class, "turret");

        intake  = hardwareMap.get(DcMotor.class, "intake");

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight  = hardwareMap.get(DcMotorEx.class, "shooterRight");

        kicker  = hardwareMap.get(Servo.class, "kicker");



        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);

        int velocity = 0;


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            double x2 = gamepad2.left_stick_x;
            turret.setPower(x2);


            if (gamepad2.right_trigger > 0.2f) {
                kicker.setPosition(1);
            } else {
                kicker.setPosition(0);
            }

            if (gamepad2.left_trigger > 0.2f){
                shooterLeft.setPower(-1);
                shooterRight.setPower(-1);
            } else if(gamepad2.left_bumper){
                shooterLeft.setPower(1);
                shooterRight.setPower(1);
            } else {
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
            }

            if (gamepad2.a){
                intake.setPower(1);
            } else if(gamepad2.b){
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            //  -1150 Short
            //  -1330 Medium
            //  -1593 Long

            if(gamepad2.dpad_left){
                //short
                velocity = (-1170);
            } else if (gamepad2.dpad_right & gamepad2.left_bumper){
                //long
                velocity = (-1593);
            }else if(gamepad2.dpad_up & gamepad2.left_bumper){
                //medium
                velocity = (-1350);
            } else if(gamepad2.dpad_down & gamepad2.left_bumper) {
                //off
                velocity = 0;
            }
            if(gamepad2.rightBumperWasPressed()){
                velocity = -(velocity);
            }
            shooterLeft.setVelocity(velocity);
            shooterRight.setVelocity(velocity);



            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            telemetry.addData("left stick x", gamepad1.left_stick_x);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftPower = frontLeftPower / 1.25;
            backLeftPower = backLeftPower / 1.25;
            frontRightPower = frontRightPower / 1.25;
            backRightPower = backRightPower / 1.25;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            telemetry.addData("y",y);
            telemetry.addData("x",x);
            telemetry.addData("rx",rx);
            telemetry.addData("frontLeftPower",frontLeftPower);
            telemetry.addData("backLeftPower",backLeftPower);
            telemetry.addData("frontRightPower",frontRightPower);
            telemetry.addData("backRightPower",backRightPower);
            telemetry.update();


        }
    }
}
