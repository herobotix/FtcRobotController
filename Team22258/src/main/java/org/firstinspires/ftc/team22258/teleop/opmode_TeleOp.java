package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "Opmode (TeleOp) [1.0.17]")
public class opmode_TeleOp extends LinearOpMode {

  private DcMotor Arm;
  private DcMotor FLMotor;
  private DcMotor FRMotor;
  private DcMotor BLMotor;
  private DcMotor BRMotor;
  private Servo LClaw;
  private Servo RClaw;

  double Twarm;
  double FArmInput;
  
  int ClawState = 0;
  int Clawn = 0;
  
  int TwoState = 0;
  int Twon = 0;
  
  double FLMP;
  double FRMP;
  double BLMP;
  double BRMP;
  
  double MPN;
  
  double Rot;
  private IMU rIMU;
  
  @Override
  public void runOpMode() {
    //Begin
    
    //Init & Wait
      Fn_Init();
      waitForStart();
    
    //Run Opmode
      while (opModeIsActive()) {
        Fn_Move();
        Fn_Clawrm();
        Fn_Telemetry();
        Fn_Update();
      }
    
  }
  
  private void Fn_Init() {
    // Initialization Code
    
    // Map Hardware
      Arm = hardwareMap.get(DcMotor.class, "Arm");
      FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
      FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
      BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
      BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
      LClaw = hardwareMap.get(Servo.class, "LClaw");
      RClaw = hardwareMap.get(Servo.class, "RClaw");
      rIMU = hardwareMap.get(IMU.class, "rIMU");
    
    // Set Motor Behaviors
      Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FLMotor.setDirection(DcMotor.Direction.FORWARD);
      FRMotor.setDirection(DcMotor.Direction.REVERSE);
      BLMotor.setDirection(DcMotor.Direction.REVERSE);
      BRMotor.setDirection(DcMotor.Direction.REVERSE);
    
    // Set Robot Orientation (IMU)
      IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
      RevHubOrientationOnRobot.LogoFacingDirection.UP,
      RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
      rIMU.initialize(parameters);
    
  }
  
  private void Fn_Move() {
    //Movement Code
    
    // Get/Reset Robot Rotation Value
      if (gamepad1.start) {rIMU.resetYaw();}
      Rot = rIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    
    // Inputs
      powHead = gamepad1.left_stick_x*Math.sin(Rot) + gamepad1.left_stick_y*Math.cos(Rot);
      powSide = gamepad1.left_stick_x*Math.cos(Rot) - gamepad1.left_stick_y*Math.sin(Rot);
      powTurn = gamepad1.right_stick_x;
    
    // Drive & Strafe & Rotate
      FLMP = powHead + powSide - powTurn;
      FRMP = powHead - powSide + powTurn;
      BLMP = powHead - powSide - powTurn;
      BRMP = powHead + powSide + powTurn;
    
    // Power Control
      MPN = (
        Math.max(1,
          Math.max(
            Math.max( Math.abs(FLMP), Math.abs(FRMP) ), 
            Math.max( Math.abs(BLMP), Math.abs(BRMP) ) 
          )
        )
      );
      if (MPN > 1){
        BRMP /= MPN;
        BLMP /= MPN;
        FRMP /= MPN;
        FLMP /= MPN;
      }
    
    // Trigger Motors
      FLMotor.setPower(FLMP);
      FRMotor.setPower(FRMP);
      BLMotor.setPower(BLMP);
      BRMotor.setPower(BRMP);
    
  }
  
  private void Fn_Clawrm() {
    // Claw & Arm Code
    
    // Run Toggle Functions
      Fn_Twoggle();
      Fn_Clawggle();
    
    FArmInput = 0.8*(Twarm.left_trigger - Twarm.right_trigger);
  }

  /**
   * Describe this function...
   */
  private void Fn_Clawggle() {
    if (ClawState == 0 && Twarm.left_bumper) {
      // claw closed
      ClawState = 1;
      // claw open
    } else if (ClawState == 1 && !Twarm.left_bumper) {
      ClawState = 2;
      // claw open
    } else if (ClawState == 2 && Twarm.left_bumper) {
      // claw open
      ClawState = 3;
      // claw closed
    } else if (ClawState == 3 && !Twarm.left_bumper) {
      ClawState = 0;
      // claw closed
    }
    if (ClawState == 0 || ClawState == 3) {
      Clawn = 0;
    } else {
      Clawn = 1;
    }
  }
  
  private void Fn_Twoggle() {
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
    Twarm = ((TwoState != 0 && TwoState != 3)?gamepad2:gamepad1);
  }

  /**
   * Describe this function...
   */
  private void Fn_Telemetry() {
    telemetry.addData("LStickX", gamepad1.left_stick_x);
    telemetry.addData("LStickY", gamepad1.left_stick_y);
    telemetry.addData("RStickX", gamepad1.right_stick_x);
    telemetry.addData("MPN", MPN);
    telemetry.addData("FLMP", FLMP);
    telemetry.addData("FLMP", FRMP);
    telemetry.addData("BLMP", BLMP);
    telemetry.addData("BRMP", BRMP);
    telemetry.addData("▲", gamepad1.dpad_up ? 1 : 0);
    telemetry.addData("▼", gamepad1.dpad_down ? 1 : 0);
    telemetry.addData("◄", gamepad1.dpad_left ? 1 : 0);
    telemetry.addData("2 Controllers", Twon);
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
  private void Fn_Update() {
    Arm.setPower(FArmInput);
    LClaw.setPosition(0 == Clawn ? 0.75 : 0.25);
    RClaw.setPosition(1 == Clawn ? 0.75 : 0.25);
    FLMP = 0;
    FRMP = 0;
    BLMP = 0;
    BRMP = 0;
  }
  
}