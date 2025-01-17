package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


@TeleOp(name = "Opmode (TeleOp) [1.0.17]")
public class opmode_TeleOp extends LinearOpMode {

  private DcMotor Arm;
  private DcMotor FLMotor;
  private DcMotor FRMotor;
  private DcMotor BLMotor;
  private DcMotor BRMotor;
  private Servo LClaw;
  private Servo RClaw;

  double FArmInput;
  
  int ClawState = 0;
  int Clawn = 0;
  
  int TwoState = 0;
  int Twon = 0;
  
  double FLMotorPower;
  double FRMotorPower;
  double BLMotorPower;
  double BRMotorPower;
  
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
      while (opModeIsActive()) {
		F_Misc();
        F_Move();
        F_Clawrm();
        F_Telemetry();
        F_Update();
      }
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
  }

  private void F_Misc() {
    if (gamepad1.options) { //Reset facing direction to robot's orientation.
		imu.resetYaw();
	}
	
  }
  
private void F_Twoggle() {
    if (TwoState == 0 && gamepad1.dpad_left) {
      // Two off
      TwoState = 1;
      // Two on
    } else if (TwoState == 1 && !gamepad1.dpad_left) {
      TwoState = 2;
      // Two on
    } else if (TwoState == 2 && gamepad1.dpad_left) {
      // Two on
      TwoState = 3;
      // Two off
    } else if (TwoState == 3 && !gamepad1.dpad_left) {
      TwoState = 0;
      // Two off
    }
    if (TwoState == 0 || TwoState == 3) {
      Twon = 0;
    } else {
      Twon = 1;
    }
  }
  
  /**
   * Describe this function...
   */
  private void F_Move() {
    if (true) {
      // Drive
      FLMotorPower += gamepad1.left_stick_y * 1.0;
      FRMotorPower += gamepad1.left_stick_y * 1.0;
      BLMotorPower += gamepad1.left_stick_y * 1.0;
      BRMotorPower += gamepad1.left_stick_y * 1.0;
    }
    if (true) {
      // Strafe
      FLMotorPower += gamepad1.left_stick_x * -1.0;
      FRMotorPower += gamepad1.left_stick_x * 1.0;
      BLMotorPower += gamepad1.left_stick_x * 1.0;
      BRMotorPower += gamepad1.left_stick_x * -1.0;
    }
    if (true) {
      // Rotate
	  double Rick = ((Twon == 1)?gamepad2:gamepad1).right_stick_x;
      FLMotorPower += Rick * -1.0;
      FRMotorPower += Rick * 1.0;
      BLMotorPower += Rick * -1.0;
      BRMotorPower += Rick * 1.0;
    }
    if (true) {
      // Power Control
		telemetry.addData("FLMotorPower OLD", FLMotorPower);
		telemetry.addData("FLMotorPower OLD", FRMotorPower);
		telemetry.addData("BLMotorPower OLD", BLMotorPower);
		telemetry.addData("BRMotorPower OLD", BRMotorPower);
      MotorPowerNormalizer = ( 
		Math.max( 
		  Math.max( Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_y) ), 
		  Math.abs(gamepad1.right_stick_x) 
		)/ ( Math.max( 
		  Math.max( Math.abs(FLMotorPower), Math.abs(FRMotorPower) ), 
		  Math.max( Math.abs(BLMotorPower), Math.abs(BRMotorPower) ) 
		) ) 
	  );
      FLMotorPower = (FLMotorPower * MotorPowerNormalizer);
      FRMotorPower = (FRMotorPower * MotorPowerNormalizer);
      BLMotorPower = (BLMotorPower * MotorPowerNormalizer);
      BRMotorPower = (BRMotorPower * MotorPowerNormalizer);
    }
  }

  /**
   * Describe this function...
   */
  private void F_Clawrm() {
    F_Clawggle();
    FArmInput = 0.8*(gamepad2.left_trigger - gamepad2.right_trigger);
  }

  /**
   * Describe this function...
   */
  private void F_Clawggle() {
    if (ClawState == 0 && gamepad2.left_bumper) {
      // claw closed
      ClawState = 1;
      // claw open
    } else if (ClawState == 1 && !gamepad2.left_bumper) {
      ClawState = 2;
      // claw open
    } else if (ClawState == 2 && gamepad2.left_bumper) {
      // claw open
      ClawState = 3;
      // claw closed
    } else if (ClawState == 3 && !gamepad2.left_bumper) {
      ClawState = 0;
      // claw closed
    }
    if (ClawState == 0 || ClawState == 3) {
      Clawn = 0;
    } else {
      Clawn = 1;
    }
  }

  /**
   * Describe this function...
   */
  private void F_Telemetry() {
	telemetry.addData("LStickX", gamepad1.left_stick_x);
	telemetry.addData("LStickY", gamepad1.left_stick_y);
	telemetry.addData("RStickX", gamepad1.right_stick_x);
	telemetry.addData("MPN", MotorPowerNormalizer);
	telemetry.addData("FLMotorPower", FLMotorPower);
	telemetry.addData("FLMotorPower", FRMotorPower);
	telemetry.addData("BLMotorPower", BLMotorPower);
	telemetry.addData("BRMotorPower", BRMotorPower);
    telemetry.addData("▲", gamepad1.dpad_up ? 1 : 0);
    telemetry.addData("▼", gamepad1.dpad_down ? 1 : 0);
    telemetry.addData("◄", gamepad1.dpad_left ? 1 : 0);
    telemetry.addData("2 Controllers", Twon ? 1 : 0);
	telemetry.addData("ClawState", ClawState);
    telemetry.addData("Clawn", Clawn);
    telemetry.addData("FArmInput", FArmInput);
    telemetry.addData("LClawPos", LClaw.getPosition());
    telemetry.addData("RClawPos", RClaw.getPosition());
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void F_Update() {
    FLMotor.setPower(FLMotorPower);
    FRMotor.setPower(FRMotorPower * -1);
    BLMotor.setPower(BLMotorPower * -1);
    BRMotor.setPower(BRMotorPower * -1);
    Arm.setPower(FArmInput);
    LClaw.setPosition(0 == Clawn ? 0.75 : 0.25);
    RClaw.setPosition(1 == Clawn ? 0.75 : 0.25);
    FLMotorPower = 0;
    FRMotorPower = 0;
    BLMotorPower = 0;
    BRMotorPower = 0;
  }
  
}