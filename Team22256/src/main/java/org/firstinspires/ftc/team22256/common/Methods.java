package org.firstinspires.ftc.team22256.common;

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


public class Methods{

    private DcMotorEx frontRight,frontLeft,backLeft,backRight;
    private DcMotor turret;
    private CRServo kicker;
    private DcMotor intake;
    private DcMotorEx shooterLeft, shooterRight;

    ElapsedTime timer = new ElapsedTime();

    private final double I2T = 0; //Inches to ticks
    private final double SI2T = 0; //Strafe inches to ticks
    private final double D2T = 0;

    public Methods(HardwareMap hardwareMap){


        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        turret = hardwareMap.get(DcMotor.class, "turret");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        kicker = hardwareMap.get(CRServo.class, "kicker");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void DriveToPosition(double inches, double power){

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks  = (int)(Math.round(inches * I2T));

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(power);
        frontRight.setVelocity(power);
        backLeft.setVelocity(power);
        backRight.setVelocity(power);

        while(frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
        }


        frontLeft.setVelocity(0);
        frontRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);
    }




    public void strafeToPosition(double inches, double power) {

        //Positive inches strafes right

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int) (Math.round(inches * SI2T));

        frontLeft.setTargetPosition(-ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(-ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(power);
        frontRight.setVelocity(power);
        backLeft.setVelocity(power);
        backRight.setVelocity(power);

        while(frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
        }


        frontLeft.setVelocity(0);
        frontRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);
    }


    public void Drive_intake(double inches, double power){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticks = (int) (Math.round(inches * I2T));

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(power);
        frontRight.setVelocity(power);
        backLeft.setVelocity(power);
        backRight.setVelocity(power);
        intake.setPower(0.5);

        while(frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){
        }

        frontLeft.setVelocity(0);
        frontRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);
        intake.setPower(0);
    }

    public void turn(double degrees,double power){

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        int ticks = (int) (Math.round(degrees * D2T));

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(-ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(-ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setVelocity(power);
        frontRight.setVelocity(power);
        backLeft.setVelocity(power);
        backRight.setVelocity(power);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
        }

        frontLeft.setVelocity(0);
        frontRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);

    }


    public void SetShooterVelocity(double velocity){

        shooterLeft.setVelocity(velocity);
        shooterRight.setVelocity(velocity);

    }
    public void intake(double seconds){
        intake.setPower(1);
        timer.reset();
        while(timer.seconds() <= seconds){
        }
        intake.setPower(0);
    }

    public void outake(double seconds){
        intake.setPower(-1);
        timer.reset();
        while(timer.seconds() <= seconds){
        }

    }

    public void kickBall(double seconds){
        kicker.setPower(1);
        timer.reset();
        while(timer.seconds() <= seconds){
        }
        kicker.setPower(0);
    }

}