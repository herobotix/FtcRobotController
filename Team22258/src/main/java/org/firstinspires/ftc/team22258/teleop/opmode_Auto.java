package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


@Autonomous(name = "Opmode (Auto) [1.0.0]") //[1.0.17]
public class opmode_TeleOp extends LinearOpMode {

  private DcMotor Arm;
  private DcMotor FLMotor;
  private DcMotor FRMotor;
  private DcMotor BLMotor;
  private DcMotor BRMotor;
  private Servo LClaw;
  private Servo RClaw;

  double FLMotorPower;
  double FRMotorPower;
  double BLMotorPower;
  double BRMotorPower;
  
  double Lynn;
  double Lake;
  double Rick;
  double FArmInput
  int Clawn;
  double Naptime;
  
  double MotorPowerNormalizer;

  private IMU imu;
  
  @Override
  public void runOpMode() {
    Arm = hardwareMap.get(DcMotor.class, "Arm");
    FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
    FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
    BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
    BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
    LClaw = hardwareMap.get(Servo.class, "LClaw");
    RClaw = hardwareMap.get(Servo.class, "RClaw");

      F_IMU();
	  F_Startup();
      waitForStart();
    if (opModeIsActive()) {
      F_Move();
      F_NewArmon();
      F_Clawrm();
      F_Pause();
    }
  }

/**
   * Describe this function...
   */
  private void F_IMU() {
	// Retrieve the IMU from the hardware map
	imu = hardwareMap.get(IMU.class, "imu");
	// Adjust the orientation parameters to match your robot
	IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
			RevHubOrientationOnRobot.LogoFacingDirection.UP,
			RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
	// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
	imu.initialize(parameters);
  }

  /**
   * Describe this function...
   */
  private void F_Startup() {
    Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Clawn = 0;
  }

  /**
   * Describe this function...
   */
  private void F_Move(double Lynn, double Lake, double Rick) {
    if (true) {
      // Drive
      FLMotorPower += Lynn * 1.0;
      FRMotorPower += Lynn * 1.0;
      BLMotorPower += Lynn * 1.0;
      BRMotorPower += Lynn * 1.0;
    }
    if (true) {
      // Strafe
      FLMotorPower += Lake * -1.0;
      FRMotorPower += Lake * 1.0;
      BLMotorPower += Lake * 1.0;
      BRMotorPower += Lake * -1.0;
    }
    if (true) {
      // Rotate
      FLMotorPower += Rick * -1.0;
      FRMotorPower += Rick * 1.0;
      BLMotorPower += Rick * -1.0;
      BRMotorPower += Rick * 1.0;
    }
    if (true) {
	  
	    //telemetry
	      telemetry.addData("FLMotorPower OLD", FLMotorPower);
		  telemetry.addData("FLMotorPower OLD", FRMotorPower);
		  telemetry.addData("BLMotorPower OLD", BLMotorPower);
		  telemetry.addData("BRMotorPower OLD", BRMotorPower);
		  telemetry.update();
	  
      MotorPowerNormalizer = ( // Power Control
		Math.max( 
		  Math.max( Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_y) ), 
		  Math.abs(gamepad1.right_stick_x) 
		)/ ( Math.max( 
		  Math.max( Math.abs(FLMotorPower), Math.abs(FRMotorPower) ), 
		  Math.max( Math.abs(BLMotorPower), Math.abs(BRMotorPower) ) 
		) ) 
	  );
	  
	  FLMotor.setPower(FLMotorPower * MotorPowerNormalizer);
      FRMotor.setPower(FRMotorPower * -MotorPowerNormalizer);
      BLMotor.setPower(BLMotorPower * -MotorPowerNormalizer);
      BRMotor.setPower(BRMotorPower * -MotorPowerNormalizer);
	  
	    //telemetry
		  telemetry.addData("Lynn", Lynn);
		  telemetry.addData("Lake", Lake);
		  telemetry.addData("Rick", Rick);
		  telemetry.addData("MPN", MotorPowerNormalizer);
		  telemetry.addData("FLMotorPower", FLMotorPower);
		  telemetry.addData("FLMotorPower", FRMotorPower);
		  telemetry.addData("BLMotorPower", BLMotorPower);
		  telemetry.addData("BRMotorPower", BRMotorPower);
		  telemetry.update();
	  
    }
  }

  /**
   * Describe this function...
   */
  private void F_Clawrm(int Clawn) {
    LClaw.setPosition(0 == Clawn ? 0.75 : 0.25);
    RClaw.setPosition(1 == Clawn ? 0.75 : 0.25);
	
	//telemetry
      telemetry.addData("Clawn", Clawn);
      telemetry.addData("LClawPos", LClaw.getPosition());
      telemetry.addData("RClawPos", RClaw.getPosition());
      telemetry.update();
	
  }

  /**
   * Describe this function...
   */
  private void F_NewArmon(double FArmInput) {
    Arm.setPower(FArmInput);
	
	//telemetry
	  telemetry.addData("FArmInput", FArmInput);
	  telemetry.update();
	
  }

  /**
   * Describe this function...
   */
  private void F_Pause(double Naptime) {
	  sleep((long)(Naptime*1000));
  }
  
}