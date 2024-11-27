package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp

public class Drivetraintest extends LinearOpMode {
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotorSimple ArmMotor;
    private DcMotorSimple IntakeArmMotor;
    private Servo ClawServo;
    private CRServo IntakeServo;
    //private Blinker control_Hub;
    //private IMU imu;
    
    //initializing stick axis and power level variables
    private boolean ClawOpen, ClawChange;
    private double LSy, LSx, RSx;
    private double FRPwr, BRPwr, FLPwr, BLPwr, MaxPwr;

    public void runOpMode() {
        // Initialize the hardware variables.
        FLMotor  = hardwareMap.get(DcMotor.class, "FLMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor"); 
        FRMotor  = hardwareMap.get(DcMotor.class, "FRMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor"); 
        ArmMotor= hardwareMap.get(DcMotorSimple.class, "ArmMotor");
        IntakeArmMotor= hardwareMap.get(DcMotorSimple.class, "IntakeArmMotor");
        ClawServo= hardwareMap.get(Servo.class, "ClawServo");
        IntakeServo= hardwareMap.get(CRServo.class, "IntakeServo");
        
        // Stop and reset encoders
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Set motors to use encoders
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); 
        
        // Initialize Claw State
        ClawOpen=false;
        ClawChange=true;
        
        // Initialize Intake State
        IntakeServo.setDirection(CRServo.Direction.REVERSE);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            LSy=gamepad1.left_stick_y;
            LSx=-gamepad1.left_stick_x;
            RSx=-gamepad1.right_stick_x;
            telemetry.addData("LSy", LSy);
            telemetry.addData("LSx", LSx);
            telemetry.addData("RSx", RSx);
            
            MaxPwr=Math.max(Math.abs(LSy)+Math.abs(LSx)+Math.abs(RSx), 1);
            FLPwr=(LSy+LSx+RSx)/MaxPwr;
            BLPwr=(LSy-LSx+RSx)/MaxPwr;
            FRPwr=(LSy-LSx-RSx)/MaxPwr;
            BRPwr=(LSy+LSx-RSx)/MaxPwr;            
            
             // Handle state transitions based on gamepad input
            FLMotor.setPower(FLPwr); 
            BLMotor.setPower(BLPwr);
            FRMotor.setPower(FRPwr); 
            BRMotor.setPower(BRPwr); 
            
            //Raise and lower the Arm
            ArmMotor.setPower(-gamepad1.right_stick_y);
            telemetry.addData("RSy", gamepad1.right_stick_y);
            
            
            //Raise and lower the Intake Arm
            if(gamepad1.left_trigger>0) IntakeArmMotor.setPower(gamepad1.left_trigger);
            else if(gamepad1.right_trigger>0) IntakeArmMotor.setPower(-gamepad1.right_trigger);
            else IntakeArmMotor.setPower(0.0);
            telemetry.addData("LTrigger", gamepad1.left_trigger);
            telemetry.addData("RTrigger", gamepad1.right_trigger);
            
            //Open and Close Claw
            if(gamepad1.a && ClawChange) {
                if(ClawOpen) {
                    ClawServo.setPosition(0.7);
                    ClawOpen=false;
                } else{
                    ClawServo.setPosition(0.4);
                    ClawOpen=true;
                }
                ClawChange=false;
            } else if (!gamepad1.a && !ClawChange) ClawChange=true;
            
            if(gamepad1.left_bumper) IntakeServo.setPower(-1);
            else if(gamepad1.right_bumper) IntakeServo.setPower(1);
            else IntakeServo.setPower(0);
            
            
            telemetry.update();
        }
       
       
  
    }
}