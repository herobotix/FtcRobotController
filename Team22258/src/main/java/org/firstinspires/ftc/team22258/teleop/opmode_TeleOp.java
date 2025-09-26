package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "Opmode (TeleOp) [1.1.1]")
public class opmode_TeleOp extends LinearOpMode {
  
  double Rot;
  private IMU rIMU;
  
  private DcMotor FLMotor;
  double FLMP;
  
  private DcMotor FRMotor;
  double FRMP;
  
  private DcMotor BLMotor;
  double BLMP;
  
  private DcMotor BRMotor;
  double BRMP;
  
  int fieldCentric = 2;
  double MPN;
  
  private DcMotor ItkMotor;
  double ItkMP;
  
  private DcMotor OtkMotor;
  double OtkMP;
  
  @Override
  public void runOpMode() {
    //Begin
    
    //Init & Wait
      Fn_Init();
      waitForStart();
      
    //Run Opmode
      while (opModeIsActive()) {
        Fn_Move();
        Fn_IOtk();
        Fn_Telemetry();
        Fn_LoopEnd();
      }
      
  }
  
  private void Fn_Init() {
    // Initialization Code
    
    // Map Hardware
      FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
      FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
      BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
      BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
      ItkMotor = hardwareMap.get(DcMotor.class, "ItkMotor");
      OtkMotor = hardwareMap.get(DcMotor.class, "OtkMotor");
      rIMU = hardwareMap.get(IMU.class, "rIMU");
      
    // Set Motor Behaviors
      FLMotor.setDirection(DcMotor.Direction.FORWARD);
      FRMotor.setDirection(DcMotor.Direction.REVERSE);
      BLMotor.setDirection(DcMotor.Direction.REVERSE);
      BRMotor.setDirection(DcMotor.Direction.REVERSE);
      ItkMotor.setDirection(DcMotor.Direction.FORWARD);
      OtkMotor.setDirection(DcMotor.Direction.FORWARD);
      
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
      
    // Toggle Field-Centrism
      fieldCentric = ((fieldCentric == 0 || fieldCentric == 2)&&!gamepad1.dpad_up)?
          (fieldCentric+1):
          (((fieldCentric == 1 || fieldCentric == 3)&&gamepad1.dpad_up)?
              (3-fieldCentric):
              (fieldCentric)
          )
      ;
      
    // Inputs
      double powHead, powSide, powTurn;
      if (fieldCentric > 1) {
          powHead = gamepad1.left_stick_x * Math.sin(Rot) + gamepad1.left_stick_y * Math.cos(Rot);
          powSide = gamepad1.left_stick_x * Math.cos(Rot) - gamepad1.left_stick_y * Math.sin(Rot);
      } else {
          powHead = gamepad1.left_stick_y;
          powSide = gamepad1.left_stick_x;
      }
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
  
  private void Fn_IOtk() {
    //Intake/Outtake Code
    
    //Intake
      ItkMP = gamepad1.right_stick_y;
      ItkMotor.setPower(ItkMP);
      
    //Outtake
      OtkMP = gamepad1.right_trigger;
      OtkMotor.setPower(OtkMP);
      
  }
  
  private void Fn_Telemetry() {
    //Telemetry Data
    
    //Gamepad 1
      telemetry.addData("LStickX", gamepad1.left_stick_x);
      telemetry.addData("LStickY", gamepad1.left_stick_y);
      telemetry.addData("RStickX", gamepad1.right_stick_x);
      telemetry.addData("RStickY", gamepad1.right_stick_y);
      telemetry.addData("▲", gamepad1.dpad_up ? 1 : 0);
      /*telemetry.addData("▼", gamepad1.dpad_down ? 1 : 0);
      telemetry.addData("◄", gamepad1.dpad_left ? 1 : 0);*/
      
    //Movement
      telemetry.addData("MPN", MPN);
      telemetry.addData("FLMP", FLMP);
      telemetry.addData("FRMP", FRMP);
      telemetry.addData("BLMP", BLMP);
      telemetry.addData("BRMP", BRMP);
      
    //IO-take
      telemetry.addData("Input Motor", ItkMP);
      telemetry.addData("Outtake Motor", OtkMP);
      
    //Misc
      telemetry.addData("Field-Centric", (fieldCentric>1));
      
    //End Code
      telemetry.update();
      
  }
  
  private void Fn_LoopEnd() {
    //Loop End Code
      
      FLMP = 0;
      FRMP = 0;
      BLMP = 0;
      BRMP = 0;
      ItkMP = 0;
      OtkMP = 0;
  }
  
}