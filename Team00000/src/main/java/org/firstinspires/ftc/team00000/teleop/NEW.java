package org.firstinspires.ftc.team00000.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team00000.ChassisMecanum;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="NEW", group="teleop")
//@Disabled
public class NEW extends LinearOpMode {
    private DcMotorEx BLMotor;
    private DcMotorEx BRMotor;
    private DcMotorEx FLMotor;
    private DcMotorEx FRMotor;
    //private IMU imu;

    private double integralSum = 0;
    private double Kp = 0;
    private double Ki = 0;
    private double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    //initializing stick axis and power level variables
    private double LSy, LSx, RSx;
    private double FRPwr, BRPwr, FLPwr, BLPwr, MaxPwr;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables.
        FLMotor  = hardwareMap.get(DcMotorEx.class, "FLMotor");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BLMotor");
        FRMotor  = hardwareMap.get(DcMotorEx.class, "FRMotor");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BRMotor");

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
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double FLpower = PIDControl(100, FLMotor.getCurrentPosition());
            FLMotor.setPower(FLpower);
            
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
            
            telemetry.update();
        }
       
       
  
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

}