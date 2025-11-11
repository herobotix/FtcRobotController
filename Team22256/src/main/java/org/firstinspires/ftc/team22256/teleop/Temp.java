
package org.firstinspires.ftc.team22256.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team22256.common.Robot;
import org.firstinspires.ftc.team22256.common.Intake;
import org.firstinspires.ftc.team22256.common.Turret;
import org.firstinspires.ftc.team22256.common.Drivetrain;



@TeleOp

public class Temp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontRight,backRight,frontLeft,backLeft;
    private DcMotorEx shooterLeft, shooterRight;



    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.


        frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");

        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight  = hardwareMap.get(DcMotorEx.class, "shooterRight");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        int velocity = 0;
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

         if(gamepad1.aWasPressed()){
             velocity +=10;
         } else if (gamepad1.bWasPressed()){
             velocity += 100;
         }else if(gamepad1.xWasPressed()){
             velocity -= 10;
         }else if(gamepad1.yWasPressed()){
             velocity -= 100;
         }else if(gamepad1.dpad_up){
             velocity = 0;
         }
         shooterLeft.setVelocity(velocity);
         shooterRight.setVelocity(velocity);

         //-1480 for long
         //-1350  for short
            //-1170 for super short
            //y=(0.000372)x^2+(3.176)x+(1094)

         telemetry.addData("Velocity",velocity);
         telemetry.update();


        }
    }
}
