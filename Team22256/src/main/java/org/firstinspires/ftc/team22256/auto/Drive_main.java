/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team22256.auto;

import android.view.ContextThemeWrapper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")


public class Drive_main extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.



    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime bTime = new ElapsedTime();
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor arm1;
    private DcMotor arm2;
    private DcMotor slide;
    private Servo claw;
    private Servo wrist;
    private PIDController controller0; // Declaration of controller object
    private PIDController controller1;
    //  [] 0 is for arm. [] 1 is for extension
    public static double p[] = {-0.041,0.01};
    public static double i[] = {0,0};
    public static double d[] = {0,0};
    public static double f[] = {-0.003,-0.07};
    double pid[] = {0,0};
    double ff[] = {0,0};
    double power[] = {0,0};
    public static double target[] = {0,0};

    public static double ticks_in_degree[] = {0.8,4.687}; // Counts per revoluton / 360

    // public static double p=0,i=0,d=0; //Declaration of pid coeffients
    //public static double f=0;  //Declaration of feedforward constant
    //public static double target = 0; // Target position of arm


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
       rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
       leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        arm1 = hardwareMap.get(DcMotor.class, "arm1");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        claw = hardwareMap.get(Servo.class,"claw");
        wrist = hardwareMap.get(Servo.class,"wrist");







        controller0  = new PIDController(p[0],i[0],d[0]); // Initialization of controller and coeffients
        controller1 = new PIDController(p[1],i[1],d[1]);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());// Allows coeficents to be altered and monitored in FtcDashboard

        double changePower = 1.25;

         pid[0] = 0;
         ff[0] = 0;
         power[0] = 0;

        pid[1] = 0;
        ff[1] = 0;
        power[1] = 0;


        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        target[1] = 0;
        target[0] = -10;
        int state = 0;
        boolean button = false;



        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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

            frontLeftPower = frontLeftPower / changePower;
            backLeftPower = backLeftPower / changePower;
            frontRightPower = frontRightPower / changePower;
            backRightPower = backRightPower / changePower;

            leftFront.setPower(-frontLeftPower);
            leftBack.setPower(-backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(-backRightPower);


            if(gamepad1.right_trigger > 0.8){
                target[0] = -10;
                sleep(500);
                arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }



            if(gamepad1.right_bumper){
               wrist.setPosition(0.1);

            } else {
                wrist.setPosition(0.6);
            }
            if(gamepad1.left_bumper){
                claw.setPosition(0.65
                );
            } else {
                claw.setPosition(0.1);
            }



            int armPos = arm1.getCurrentPosition();
            pid[0] = controller0.calculate(armPos,target[0]);
            ff[0] = Math.cos(Math.toRadians(target[0] / ticks_in_degree[0])) * f[0];
            power[0] = pid[0] + ff[0];
            arm1.setPower(-power[0]);
            arm2.setPower(power[0]);
/*
            if(gamepad1.dpad_up && target[0] < 150 && target[0] >= 0){
                target[0] = target[0] + 7;
            }else if(gamepad1.dpad_up && target[0] <= 300 && target[0] >= 150) {
                target[0] = target[0] + 3.5;  //up for arm
            }else if(gamepad1.dpad_down && target[0] >=150 && target[0] <= 300){
                target[0] = target[0] - 6;
            }else if(gamepad1.dpad_down && target[0] >= 0 && target[0] <= 150){
                target[0] = target[0] - 4;  //down for arm
            }
*/
            if(gamepad1.dpad_up){
                target[0] = 280;
            } else if(gamepad1.dpad_down && target[0] <= 300 && target[0] >= 270){
                target[0] = 150;
                sleep(100);
                target[0] = 100;
                sleep(300);
                target[0] = 50;
                sleep(400);
                target[0] = 0;
            }

            if(target[0] >= 300){
                target[0] = 300;
            }
            if(target[0] < 0){
                target[0] = 0;
            }
            if(target[0] < 240 && target[1] > 2500 ){
                target[0] = 240;
            }


            int SPos = slide.getCurrentPosition();
            pid[1] = controller1.calculate(SPos,target[1]);
            ff[1] = Math.cos(Math.toRadians(target[1] / ticks_in_degree[1])) * f[1];
            power[1] = pid[1] + ff[1];
            slide.setPower(power[1]);


            if (gamepad1.x && target[0] >= 0 && target[0] <= 50){
                target[1] = target[1] - 30;
            }else if (gamepad1.y && target[0] >= 0 && target[0] <= 50){
                target[1] = target[1] + 30;
            }

            if(target[0] <= 0 && target[0] >= -30 && target[1] <= -1021){
                target[1] =-1020;
            }



            if (gamepad1.x && target[0] <= 300 && target[0] > 50){
                target[1] = -3300;
            }else if (gamepad1.y && target[0] <= 300 && target[0] > 50){
                target[1] = -10;
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.addData("arm1",arm1.getCurrentPosition());
            telemetry.addData("arm2",arm2.getCurrentPosition());
            telemetry.addData("slide target",target[1]);
            telemetry.addData("arm target",target[0]);
            telemetry.addData("right bumper",gamepad1.right_bumper);
            telemetry.addData("x", gamepad1.x);
            telemetry.addData("power",arm1.getPower());
            telemetry.addData("wrist", wrist.getPosition());
            telemetry.addData("claw",claw.getPosition());

            telemetry.update();
        }
    }}
