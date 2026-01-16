package org.firstinspires.ftc.team22258.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team22258.teamcode.classes.LIMELIGHT;
import org.firstinspires.ftc.team22258.teamcode.classes.IOTAKE;
import com.bylazar.configurables.annotations.Configurable;


@TeleOp(name = "Opmode (TeleOp) [1.2.9]")
@Configurable
public class opmode_TeleOp extends LinearOpMode {
  
  double currentRot;
  double targetRot;
  private IMU rIMU;
  
  private DcMotor FLMotor;
  double FLMP;
  
  private DcMotor FRMotor;
  double FRMP;
  
  private DcMotor BLMotor;
  double BLMP;
  
  private DcMotor BRMotor;
  double BRMP;
  
  boolean fieldCentric = true;
  double MPN;
  
  double powHead, powSide, powTurn;
  
  private LIMELIGHT Limelight;
  private IOTAKE IOtake;
  
  @Override
  public void runOpMode() {
    //Begin
    
    //Init & Wait
      Fn_Init();
      waitForStart();
      Fn_OnStart();
      
    //Run Opmode
      while (opModeIsActive()) {
        Limelight.Run(gamepad1, telemetry);
        Fn_Move();
        IOtake.IOtk(gamepad2);
        Fn_Telemetry();
        Fn_LoopEnd();
        
      }
      Fn_OnStop();
      
  }
  
  
  private void Fn_Init() {
    // Initialization Code
    
    // Map Hardware
      FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
      FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
      BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
      BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
      rIMU = hardwareMap.get(IMU.class, "rIMU");
      
    // Set Robot Orientation (IMU)
      IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
      );
      rIMU.initialize(parameters);
      
    // Set Robot Rotation Value
      currentRot = rIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
      targetRot = currentRot;
      
    // Set Motor Behaviors
      FLMotor.setDirection(DcMotor.Direction.REVERSE);
      FRMotor.setDirection(DcMotor.Direction.FORWARD);
      BLMotor.setDirection(DcMotor.Direction.FORWARD);
      BRMotor.setDirection(DcMotor.Direction.FORWARD);
    
    // Classes
      Limelight = new LIMELIGHT();
      Limelight.Init(hardwareMap);
      IOtake = new IOTAKE();
      IOtake.Init(hardwareMap);
    
  }
  
  private void Fn_OnStart() {
    //Run On START
    
    //Limelight
      Limelight.Start();
      
  }
  
  /*private double NormalizeAngle(double iAngle) {
    return (( iAngle + Math.PI ) % ( 2 * Math.PI )) - Math.PI;
  }*/
  
  private void Fn_Move() {
    //Movement Code
    
    // Get/Reset Robot Rotation Value
      if (gamepad1.start) {rIMU.resetYaw();}
      currentRot = rIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
      
    // Inputs
      fieldCentric = gamepad1.bWasPressed() == (!fieldCentric); // Field-Centrism Toggle
      if (fieldCentric) {
        powHead = gamepad1.left_stick_x * Math.sin(currentRot) + -gamepad1.left_stick_y * Math.cos(currentRot);
        powSide = gamepad1.left_stick_x * Math.cos(currentRot) + -gamepad1.left_stick_y * Math.sin(currentRot);
        /*
          if (Limelight.getTargetLock() == LIMELIGHT.TargetLock.OFF) {
            targetRot = NormalizeAngle(targetRot + gamepad1.right_stick_x / (200) );
            powTurn = NormalizeAngle(targetRot - currentRot) / (2*Math.PI);
            //Base this on if gamepad1.right_stick_x is not 0
          } else {
            powTurn = Limelight.getPowTurn();
          }
        */
        
      } else {
        powHead = -gamepad1.left_stick_y;
        powSide = gamepad1.left_stick_x;
        /*
          powTurn = (
            (Limelight.getTargetLock() != LIMELIGHT.TargetLock.OFF)? (Limelight.getPowTurn()):
              (gamepad1.right_stick_x)
          );
        */
      }
      powTurn = (
        (Limelight.getTargetLock() != LIMELIGHT.TargetLock.OFF)? (Limelight.getPowTurn()):
        (gamepad1.right_stick_x)
      );
      
    //Processing
      Fn_MoveProcessing();
      
  }
  
  private void Fn_MoveProcessing() {
    //Movement Processing Code
      
    // Drive & Strafe & Rotate
      FLMP = powHead + powSide + powTurn;
      FRMP = powHead - powSide - powTurn;
      BLMP = powHead - powSide + powTurn;
      BRMP = powHead + powSide - powTurn;
      
    // Power Control
      MPN = (
        Math.max(
          Math.max(
            Math.max( Math.abs(FLMP), Math.abs(FRMP) ),
            Math.max( Math.abs(BLMP), Math.abs(BRMP) )
          ),
          (1)
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
  
  private void Fn_Telemetry() {
    // Telemetry Data
      
    // Movement
      telemetry.addLine("Movement ─");
      telemetry.addData("Head Power",powHead);
      telemetry.addData("Side Power",powSide);
      telemetry.addData("Turn Power",powTurn);
      telemetry.addData("Target currentRot",targetRot);
      telemetry.addData("Current currentRot", currentRot);
      telemetry.addLine();
      
    // Classes
      Limelight.doTelemetry(telemetry, fieldCentric);
      IOtake.doTelemetry(telemetry);
      
    // End Code
      telemetry.update();
      
  }
  
  private void Fn_LoopEnd() {
    //Loop End Code
      /* Zero Values */
        FLMP =
        FRMP =
        BLMP =
        BRMP =
        (0)
      ;
      Limelight.setPowTurn(0);
      IOtake.setFlywheelPower(0.);
      IOtake.setIntakeMotorPower(0.);
  }
  
  private void Fn_OnStop() {
    //Run On STOP
    
    //Limelight
    Limelight.Stop();
    
  }
  
}