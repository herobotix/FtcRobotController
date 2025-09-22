package org.firstinspires.ftc.team22256.methods;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    public void strafe_intake(double inches, double power){
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
        intake_2.setPower(-0.4);

        while(leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()){
        }

        leftFront.setVelocity(0);
        rightFront.setVelocity(0);
        leftBack.setVelocity(0);
        rightBack.setVelocity(0);
        intake_2.setPower(0);
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
        wrist.setPosition(0);
        intake.setPosition(1);
    }
    public void intake(double seconds){
        timer.reset();
        intake_2.setPower(-0.5);
        while(timer.seconds() <= seconds){
        }
        intake_2.setPower(0);
    }
    public void outtake(double seconds){
    timer.reset();
    intake_2.setPower(0.5);
    while(timer.seconds() < seconds){
    }
    intake_2.setPower(0);
    }

    public void setTarget( double Target, boolean top){
        double p = 0.05, i = 0, d = 0;
        PIDController controller0 = new PIDController(p,i,d);
        double f = -0.05;
        double pid = 0;
        double ff = 0;
        double power = 0;
        double ticks_in_degree = 5.9744;
        double target = Target;

        while(Math.abs(target - slide.getCurrentPosition()) > 2){
            controller0.setPID(p, i, d);
            int slidePos = slide.getCurrentPosition();
            pid = controller0.calculate(slidePos, target);
            ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            power = pid + ff;
            slide.setPower(power);
        }
        if(top = true){
            slide.setPower(-0.1124);
        } else {
            slide.setPower(-0.0999);
        }
    }
}
