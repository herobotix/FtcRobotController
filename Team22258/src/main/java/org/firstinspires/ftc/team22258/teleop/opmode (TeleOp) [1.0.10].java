package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;


/*
Replaced MotorPowerNormalizer function with a shorter version.
*/


@TeleOp(name = "Opmode (TeleOp) [1.0.10]")
public class opmode__TeleOp___1_0_10_ extends LinearOpMode {

  private DcMotor Arm;
  private DcMotor FLMotor;
  private DcMotor FRMotor;
  private DcMotor BLMotor;
  private DcMotor BRMotor;
  private DcMotor Lift;
  private Servo LClaw;
  private Servo RClaw;
  private Servo Wrist;

  double ArmInput;
  int ArmState;
  int Armon;
  
  int ClawState;
  int Clawn;
  
  int WristState;
  int Wriston;
  
  int ArmLiftPower;
  int ArmTiltPower;
  
  int FLMotorPower;
  int FRMotorPower;
  int BLMotorPower;
  int BRMotorPower;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue Comment
   * Blocks show where to place Initialization code (runs once, after touching the DS INIT
   * button, and before touching the DS Start arrow), Run code (runs once, after touching
   * Start), and Loop code (runs repeatedly while the OpMode is active, namely not Stopped).
   */
  @Override
  public void runOpMode() {
    Arm = hardwareMap.get(DcMotor.class, "Arm");
    FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
    FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
    BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
    BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
    Lift = hardwareMap.get(DcMotor.class, "Lift");
    LClaw = hardwareMap.get(Servo.class, "LClaw");
    RClaw = hardwareMap.get(Servo.class, "RClaw");
    Wrist = hardwareMap.get(Servo.class, "Wrist");

      Startup();
      waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
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
  private void Startup() {
    Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    ClawState = 0;
    WristState = 0;
    ArmState = 0;
    Clawn = 0;
    Wriston = 0;
  }

  /**
   * Describe this function...
   */
  private void F_Move() {
    double MotorPowerNormalizer;

    if (true) {
      // Drive
      FLMotorPower += gamepad1.left_stick_y * 1;
      FRMotorPower += gamepad1.left_stick_y * 1;
      BLMotorPower += gamepad1.left_stick_y * 1;
      BRMotorPower += gamepad1.left_stick_y * 1;
    }
    if (true) {
      // Strafe
      FLMotorPower += gamepad1.left_stick_x * -1;
      FRMotorPower += gamepad1.left_stick_x * 1;
      BLMotorPower += gamepad1.left_stick_x * 1;
      BRMotorPower += gamepad1.left_stick_x * -1;
    }
    if (true) {
      // Rotate
      FLMotorPower += gamepad1.right_stick_x * -1;
      FRMotorPower += gamepad1.right_stick_x * 1;
      BLMotorPower += gamepad1.right_stick_x * -1;
      BRMotorPower += gamepad1.right_stick_x * 1;
    }
    if (true) {
      // Power Control
      MotorPowerNormalizer = ( Math.max( Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_y), Math.abs(gamepad1.right_stick_x) ) / ( (double) Math.max( Math.abs(FLMotorPower), Math.abs(FRMotorPower), Math.abs(BLMotorPower), Math.abs(BRMotorPower) ) ) );
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
    F_Wristoggle();
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
   * Describe this function...
   */
  private void F_Wristoggle() {
    if (WristState == 0 && gamepad2.right_bumper) {
      // claw closed
      WristState = 1;
      // claw open
    } else if (WristState == 1 && !gamepad2.right_bumper) {
      WristState = 2;
      // claw open
    } else if (WristState == 2 && gamepad2.right_bumper) {
      // claw open
      WristState = 3;
      // claw closed
    } else if (WristState == 3 && !gamepad2.right_bumper) {
      WristState = 0;
      // claw closed
    }
    if (WristState == 0 || WristState == 3) {
      Wriston = 0;
    } else {
      Wriston = 1;
    }
  }

  /**
   * Describe this function...
   */
  private void F_Arm() {
    ArmInput = gamepad2.left_trigger - gamepad2.right_trigger;
    if (ArmInput != 0) {
      F_ToggleArm();
      ArmTiltPower += ArmInput * (Armon == 1 ? 1 : 0.8);
      // Turn Arm
    }
    ArmLiftPower += gamepad2.dpad_up ? 1 : (gamepad2.dpad_down ? -1 : 0);
    // Lift Arm
  }

  /**
   * Describe this function...
   */
  private void F_ToggleArm() {
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
    telemetry.addData("WristState", WristState);
    telemetry.addData("ArmState", ArmState);
    telemetry.addData("Clawn", Clawn);
    telemetry.addData("Wriston", Wriston);
    telemetry.addData("ArmInput", ArmInput);
    telemetry.addData("Armon", Armon);
    telemetry.addData("ArmTiltPower", ArmTiltPower);
    telemetry.addData("ArmLiftPower", ArmLiftPower);
    telemetry.addData("WristPos", Wrist.getPosition());
    telemetry.addData("LClawPos", LClaw.getPosition());
    telemetry.addData("RClawPos", RClaw.getPosition());
    telemetry.F_Update();
  }
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
    Lift.setPower(ArmLiftPower * -1);
    LClaw.setPosition(0 == Clawn ? 0.75 : 0.25);
    RClaw.setPosition(1 == Clawn ? 0.75 : 0.25);
    Wrist.setPosition(1 == Wriston ? 1 : 0.7);
    FLMotorPower = 0;
    FRMotorPower = 0;
    BLMotorPower = 0;
    BRMotorPower = 0;
    ArmTiltPower = 0;
    ArmLiftPower = 0;
  }
