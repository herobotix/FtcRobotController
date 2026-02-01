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
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Opmode (TeleOp) [1.2.12]")
@Configurable
public class opmode_TeleOp extends LinearOpMode {
  
  double robotYawRadians;
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
  private Servo indicatorLight;
  
  @Override
  public void runOpMode() {
    //Begin
    
    //Init & Wait
      OnInit();
      waitForStart();
      OnStart();
      
    //Run Opmode
      while (opModeIsActive()) {
        Limelight.Run(gamepad1, telemetry);
        Fn_Inputs();
        Fn_Move();
        IOtake.Run(gamepad2);
        doTelemetry();
        LoopEnd();
        
      }
      OnStop();
      
  }
  
  private void OnInit() {
    // Initialization Code
    
    // Map Hardware
      FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
      FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
      BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
      BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
      rIMU = hardwareMap.get(IMU.class, "rIMU");
      indicatorLight = hardwareMap.get(Servo.class, "indicatorLight");

    // Set Robot Orientation (IMU)
      IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
      );
      rIMU.initialize(parameters);
      
    // Set Robot Rotation Value
      robotYawRadians = rIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
      targetRot = robotYawRadians;
      
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
  
  private void OnStart() {
    //Run On START
    
    //Limelight
      Limelight.Start();
      
  }
  
  private void Fn_Inputs() {
    // Inputs Code
    
    // Reset Robot Rotation Value
      if (gamepad1.start) {rIMU.resetYaw();}
      
    // Inputs
      fieldCentric = gamepad1.bWasPressed() == (!fieldCentric); // Field-Centrism Toggle
      powHead = -gamepad1.left_stick_y;
      powSide = gamepad1.left_stick_x;
      powTurn = (
        (Limelight.getTargetLock() != LIMELIGHT.TargetLock .OFF )? (Limelight.getPowTurn()):
        (gamepad1.right_stick_x)
      );

      // GoBilda RGB indicator light, PWM values from GoBilda product page
      if(indicatorLight != null) {
          if (Limelight.getTargetLock() == LIMELIGHT.TargetLock.OFF) {
              indicatorLight.setPosition(0.0);
          } else if (Limelight.getTargetLock() == LIMELIGHT.TargetLock.BLUE) {
              indicatorLight.setPosition(0.611);
          } else if (Limelight.getTargetLock() == LIMELIGHT.TargetLock.RED) {
              indicatorLight.setPosition(0.277);
          }
      }
  }
  
  private void Fn_Move() {
    //Movement Code
    
    // Get Robot Rotation Value
      robotYawRadians = rIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit .RADIANS );
      
    // Field Centric
      if (fieldCentric) {
        powHead = powSide * Math.sin(robotYawRadians) + powHead * Math.cos(robotYawRadians);
        powSide = powSide * Math.cos(robotYawRadians) + powHead * Math.sin(robotYawRadians);
      }
    
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
  
  private void doTelemetry() {
    // Telemetry Data
      
    // Movement
      telemetry.addLine("Movement ─");
      telemetry.addData("Head Power",powHead);
      telemetry.addData("Side Power",powSide);
      telemetry.addData("Turn Power",powTurn);
      telemetry.addData("Target Robot Yaw",targetRot / (2*Math.PI));
      telemetry.addData("Current Robot Yaw", robotYawRadians / (2*Math.PI));
      telemetry.addLine();
      
    // Do for Classes
      Limelight.doTelemetry(telemetry, fieldCentric);
      IOtake.doTelemetry(telemetry);
      
    // Update
      telemetry.update();
      
  }
  
  private void LoopEnd() {
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
  
  private void OnStop() {
    //Run On STOP
    
    //Limelight
    Limelight.Stop();
    
  }
  
}