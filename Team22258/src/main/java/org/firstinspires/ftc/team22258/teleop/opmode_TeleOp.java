package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


@TeleOp(name = "Opmode (TeleOp) [1.0.15]")
public class opmode_TeleOp extends LinearOpMode {

  private DcMotor Arm;
  private DcMotor FLMotor;
  private DcMotor FRMotor;
  private DcMotor BLMotor;
  private DcMotor BRMotor;
  private Servo LClaw;
  private Servo RClaw;

  double ArmInput;
  int ArmState;
  int Armon;
  
  int ClawState;
  int Clawn;
  
  int ArmTiltPower;
  
  int FLMotorPower;
  int FRMotorPower;
  int BLMotorPower;
  int BRMotorPower;

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
    ClawState = 0;
    ArmState = 0;
    Clawn = 0;
	Armon = 0;
  }

  private void F_Misc() {
    if (gamepad1.options) { //Reset facing direction to robot's orientation.
		imu.resetYaw();
	}
  }

  /**
   * Describe this function...
   */
  private void F_Move() {
    double MotorPowerNormalizer;

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
      FLMotorPower += gamepad1.right_stick_x * -1.0;
      FRMotorPower += gamepad1.right_stick_x * 1.0;
      BLMotorPower += gamepad1.right_stick_x * -1.0;
      BRMotorPower += gamepad1.right_stick_x * 1.0;
    }
    if (true) {
      // Power Control
      MotorPowerNormalizer =  (double) ( Math.max( Math.max( Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_y) ), Math.abs(gamepad1.right_stick_x) ) / ( Math.max( Math.max( Math.abs(FLMotorPower), Math.abs(FRMotorPower) ), Math.max( Math.abs(BLMotorPower), Math.abs(BRMotorPower) ) ) ) );
      FLMotorPower = (int) (FLMotorPower * MotorPowerNormalizer);
      FRMotorPower = (int) (FRMotorPower * MotorPowerNormalizer);
      BLMotorPower = (int) (BLMotorPower * MotorPowerNormalizer);
      BRMotorPower = (int) (BRMotorPower * MotorPowerNormalizer);
    }
  }

  /**
   * Describe this function...
   */
  private void F_Clawrm() {
    F_Clawggle();
    F_Arm();
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
   * Moves Arm
   */
  private void F_Arm() {
    ArmInput = gamepad2.left_trigger - gamepad2.right_trigger;
    if (ArmInput != 0) {
      F_NewWriston();
      ArmTiltPower += ArmInput * (Armon == 1 ? 1 : 0.8);
      // Turn Arm
    }
  }

  /**
   * Toggles Arm
   */
  private void F_NewWriston() {
    if (ArmState == 0 && gamepad2.dpad_left) {
      // claw closed
      ArmState = 1;
      // claw open
    } else if (ArmState == 1 && !gamepad2.dpad_left) {
      ArmState = 2;
      // claw open
    } else if (ArmState == 2 && gamepad2.dpad_left) {
      // claw open
      ArmState = 3;
      // claw closed
    } else if (ArmState == 3 && !gamepad2.dpad_left) {
      ArmState = 0;
      // claw closed
    }
    if (ArmState == 0 || ArmState == 3) {
      Armon = 0;
    } else {
      Armon = 1;
    }
  }

  /**
   * Describe this function...
   */
  private void F_Telemetry() {
    telemetry.addData("▲", gamepad1.dpad_up ? 1 : 0);
    telemetry.addData("▼", gamepad1.dpad_down ? 1 : 0);
    telemetry.addData("◄", gamepad1.dpad_left ? 1 : 0);
    telemetry.addData("ClawState", ClawState);
    telemetry.addData("ArmState", ArmState);
    telemetry.addData("Clawn", Clawn);
    telemetry.addData("ArmInput", ArmInput);
    telemetry.addData("Armon", Armon);
    telemetry.addData("ArmTiltPower", ArmTiltPower);
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
    Arm.setPower(ArmTiltPower);
    LClaw.setPosition(0 == Clawn ? 0.75 : 0.25);
    RClaw.setPosition(1 == Clawn ? 0.75 : 0.25);
    FLMotorPower = 0;
    FRMotorPower = 0;
    BLMotorPower = 0;
    BRMotorPower = 0;
    ArmTiltPower = 0;
  }
  
}