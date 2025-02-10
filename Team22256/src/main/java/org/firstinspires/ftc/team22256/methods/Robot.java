package org.firstinspires.ftc.team22256.methods;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot{

    public DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public DcMotor rotator, slide;
    public Servo  wrist,intake;
    public CRServo intake_2;

    double I2T = 52.71875;
    int SI2T = 42;
    double D2T = 12.701388889;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();

    public Robot(HardwareMap hardwareMap) {

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        slide = hardwareMap.get(DcMotor.class, "slide");

        intake_2 = hardwareMap.get(CRServo.class,"intake_2");
        intake = hardwareMap.get(Servo.class,"intake");
        wrist = hardwareMap.get(Servo.class,"wrist");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    public void driveToPosition(double inches,double power) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        int ticks  = (int) (Math.round(inches * I2T));

        leftFront.setTargetPosition(-ticks);
        rightFront.setTargetPosition(-ticks);
        leftBack.setTargetPosition(ticks);
        rightBack.setTargetPosition(-ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setVelocity(power);
        rightFront.setVelocity(power);
        leftBack.setVelocity(power);
        rightBack.setVelocity(power);

        while(leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()){
}
        leftFront.setVelocity(0);
        rightFront.setVelocity(0);
        leftBack.setVelocity(0);
        rightBack.setVelocity(0);


    }

    public void strafeToPosition(double power, double inches) {

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int) (Math.round(inches * SI2T));

        leftFront.setTargetPosition(-ticks);
        rightFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(-ticks);
        rightBack.setTargetPosition(-ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setVelocity(power);
        rightFront.setVelocity(power);
        leftBack.setVelocity(power);
        rightBack.setVelocity(power);

        while(leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()){
        }


        leftFront.setVelocity(0);
        rightFront.setVelocity(0);
        leftBack.setVelocity(0);
        rightBack.setVelocity(0);
    }
    public void turn(double degrees,double power){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int ticks = (int) (Math.round(degrees * D2T));

        leftFront.setTargetPosition(ticks);
        rightFront.setTargetPosition(-ticks);
        leftBack.setTargetPosition(-ticks);
        rightBack.setTargetPosition(-ticks);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setVelocity(power);
        rightFront.setVelocity(power);
        leftBack.setVelocity(power);
        rightBack.setVelocity(power);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
        }

        leftFront.setVelocity(0);
        rightFront.setVelocity(0);
        leftBack.setVelocity(0);
        rightBack.setVelocity(0);

    }


    public void intake_pos(){
        wrist.setPosition(0.9);
        intake.setPosition(0);
    }
    public void score_pos(){
        wrist.setPosition(0.45);
        intake.setPosition(1);
    }
    public void intake(double seconds){
    timer.reset();
    intake_2.setPower(-0.5);
    if(timer.seconds() >= seconds){
        intake_2.setPower(0);
        }
    }
    public void outtake(double seconds){
    timer2.reset();
    intake_2.setPower(0.5);
    if(timer2.seconds() >= seconds){
        intake_2.setPower(0);
    }
    }



}
