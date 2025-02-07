package org.firstinspires.ftc.team22258.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "Opmode (TeleOp) [1.1.12]")
public class opmode_TeleOp extends LinearOpMode {

  private DcMotor Arm;
  private DcMotor FLM;
  private DcMotor FRM;
  private DcMotor BLM;
  private DcMotor BRM;
  private Servo LClaw;
  private Servo RClaw;

  Gamepad Twarm;
  double FArmInput;
  
  int ClawState = 0;
  int Clawn = 0;
  
  int TwoState = 0;
  int Twon = 0;
  
  double FLMP;
  double FRMP;
  double BLMP;
  double BRMP;
  
  double dstHead;
  double powHead;
  double dstSide;
  double powSide;
  double dstTurn;
  double powTurn;


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
      }
    
  }
  
  private void Fn_Init() {
  // Initialization Code
    
    // Map Hardware
      Arm = hardwareMap.get(DcMotor.class, "Arm");
      FLM = hardwareMap.get(DcMotor.class, "FLMotor");
      FRM = hardwareMap.get(DcMotor.class, "FRMotor");
      BLM = hardwareMap.get(DcMotor.class, "BLMotor");
      BRM = hardwareMap.get(DcMotor.class, "BRMotor");
      LClaw = hardwareMap.get(Servo.class, "LClaw");
      RClaw = hardwareMap.get(Servo.class, "RClaw");
      rIMU = hardwareMap.get(IMU.class, "rIMU");
    
    // Set Motor Behaviors
      Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      FLM.setDirection(DcMotor.Direction.FORWARD);
      FRM.setDirection(DcMotor.Direction.REVERSE);
      BLM.setDirection(DcMotor.Direction.REVERSE);
      BRM.setDirection(DcMotor.Direction.REVERSE);
    
    // Set Robot Orientation (IMU)
      /*IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(  // For HEROBOT
              RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
              RevHubOrientationOnRobot.UsbFacingDirection.LEFT
      ));*/
      IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(  // For BEANIEBOT
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
      powSide = gamepad1.left_stick_y*Math.sin(Rot) - gamepad1.left_stick_x*Math.cos(Rot);
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
    
    // Trigger
      FLM.setPower(FLMP);
      FRM.setPower(FRMP);
      BLM.setPower(BLMP);
      BRM.setPower(BRMP);
    
  }
  
  private void Fn_Clawrm() {
  // Claw & Arm Code
    
    // Run Toggle Functions
      Fn_Twoggle();
      Fn_Clawggle();
    
    // Trigger
      Arm.setPower(0.8*(Twarm.left_trigger - Twarm.right_trigger));  
      LClaw.setPosition(0 == Clawn ? 0.75 : 0.25);
      RClaw.setPosition(0 == Clawn ? 0.25 : 0.75);
    
  }
  
  private void Fn_Clawggle() {
  // Claw Toggler
    
    //Toggle Cycle
      if      (ClawState == 0 && !Twarm.left_bumper) {ClawState = 1;} //Release  Off
      else if (ClawState == 1 && Twarm.left_bumper)  {ClawState = 2;} //Press -► On
      else if (ClawState == 2 && !Twarm.left_bumper) {ClawState = 3;} //Release  On
      else if (ClawState == 3 && Twarm.left_bumper)  {ClawState = 0;} //Press -► Off
    
    //Output Variable
      Clawn = (ClawState == 2 || ClawState == 3)?1:0;
    
  }
  
  private void Fn_Twoggle() {
  // Dual Controller Toggler
    
    //Toggle Cycle
      if      (TwoState == 0 && !gamepad1.dpad_left) {TwoState = 1;}  //Release  Off
      else if (TwoState == 1 && gamepad1.dpad_left)  {TwoState = 2;}  //Press -► On
      else if (TwoState == 2 && !gamepad1.dpad_left) {TwoState = 3;}  //Release  On
      else if (TwoState == 3 && gamepad1.dpad_left)  {TwoState = 0;}  //Press -► Off
    
    //Output Variable
      Twarm = (TwoState == 2 || TwoState == 3)?gamepad2:gamepad1;
    
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
    telemetry.addData("FArmInput", Arm.getPower());
    telemetry.addData("LClawPos", LClaw.getPosition());
    telemetry.addData("RClawPos", RClaw.getPosition());
    telemetry.update();
  }
  
}