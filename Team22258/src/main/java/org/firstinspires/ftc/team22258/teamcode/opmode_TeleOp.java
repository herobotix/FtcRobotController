package org.firstinspires.ftc.team22258.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;


@TeleOp(name = "Opmode (TeleOp) [1.1.4]")
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
  
  boolean TargetLockGOAL = false;
  boolean TargetLockGOAL_Target = false;
  double TargetLockXRot;
  
  boolean fieldCentric = 2;
  double MPN;
  
  double powHead, powSide, powTurn;
  
  private DcMotor ItkMotor;
  double ItkMP;
  
  private DcMotor OtkMotor;
  double OtkMP;
  
  private Limelight3A limelight;
  
  @Override
  public void runOpMode() {
    //Begin
    
    //Init & Wait
      Fn_Init();
      waitForStart();
      Fn_OnStart();
      
    //Run Opmode
      while (opModeIsActive()) {
        Fn_Limelight();
        Fn_FullMove();
        Fn_MoveProcessing();
        Fn_IOtk();
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
      ItkMotor = hardwareMap.get(DcMotor.class, "ItkMotor");
      OtkMotor = hardwareMap.get(DcMotor.class, "OtkMotor");
      limelight = hardwareMap.get(Limelight3A.class, "limelight");
      rIMU = hardwareMap.get(IMU.class, "rIMU");
      
    // Set Motor Behaviors
      FLMotor.setDirection(DcMotor.Direction.FORWARD);
      FRMotor.setDirection(DcMotor.Direction.REVERSE);
      BLMotor.setDirection(DcMotor.Direction.REVERSE);
      BRMotor.setDirection(DcMotor.Direction.REVERSE);
      ItkMotor.setDirection(DcMotor.Direction.REVERSE);
      OtkMotor.setDirection(DcMotor.Direction.REVERSE);
      
    // Set Robot Orientation (IMU)
      IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
      RevHubOrientationOnRobot.LogoFacingDirection.UP,
      RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
      rIMU.initialize(parameters);
      
  }
  
  private void Fn_OnStart() {
    //Run On START
    
    //Limelight
      limelight.pipelineSwitch(0);
      limelight.start();
      
  }
  
  private void Fn_OnStop() {
    //Run On STOP
    
    //Limelight
      limelight.stop();
      
  }
  
  private void Fn_Limelight() {
    //Limelight Code
    
    //LockOn Detection
      TargetLockGOAL_Target = (gamepad1.yWasPressed())?(!TargetLockGOAL_Target):(TargetLockGOAL_Target);
    
    //Limelight IMU
      YawPitchRollAngles orientation = rIMU.getRobotYawPitchRollAngles();
      limelight.updateRobotOrientation(orientation.getYaw());
      LLResult results = limelight.getLatestResult();
      if (results != null && results.isValid()) {
        List<LLResultTypes.FiducialResult> fiducials = results.getFiducialResults();
        if (!fiducials.isEmpty()) {
          int index = 0;
          for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int aprilTagID = fiducial.getFiducialId();
            double aprilTagX = fiducial.getTargetXDegrees();
            telemetry.addData("Detection " + index + " ID:", aprilTagID);
            telemetry.addData("at X: ", aprilTagX);
            if (TargetLockGOAL&&(aprilTagID==((TargetLockGOAL_Target)?(20):(24)))) {
              TargetLockXRot = aprilTagX/360;
            } else {
              TargetLockXRot = gamepad1.right_stick_x;
            }
            index++;
          }
        } else {
          telemetry.addLine("Limelight Fiducials Empty");
        }
      } else {
        telemetry.addLine("No AprilTags Detected");
      }
      
  }
  
  private int Fn_Toggler(int ToggleState,boolean ToggleButton) {
    ToggleState = ((ToggleState == 0 || ToggleState == 2)&&!ToggleButton)?
      (ToggleState+1):
      (((ToggleState == 1 || ToggleState == 3)&&ToggleButton)?
        (3-ToggleState):
        (ToggleState)
      )
    ;
    return ToggleState;
  }
  
  private void Fn_FullMove() {
    //Player-controlled & Autonomous TeleOp Movement Code
    
    // Get/Reset Robot Rotation Value
      if (gamepad1.start) {rIMU.resetYaw();}
      Rot = rIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    
    // Toggle Field-Centrism
//      fieldCentric = ((fieldCentric == 0 || fieldCentric == 2)&&!gamepad1.dpad_up)?
//        (fieldCentric+1):
//        (((fieldCentric == 1 || fieldCentric == 3)&&gamepad1.dpad_up)?
//          (3-fieldCentric):
//          (fieldCentric)
//        )
//      ;
      fieldCentric = (gamepad1.bWasPressed())?(!fieldCentric):(fieldCentric);
      
    // Inputs
      if (fieldCentric) {
        powHead = gamepad1.left_stick_x * Math.sin(Rot) + gamepad1.left_stick_y * Math.cos(Rot);
        powSide = gamepad1.left_stick_x * Math.cos(Rot) - gamepad1.left_stick_y * Math.sin(Rot);
        powTurn = gamepad1.right_stick_x;
      } else {
        powHead = gamepad1.left_stick_y;
        powSide = gamepad1.left_stick_x;
        powTurn = (TargetLockGOAL)?(TargetLockXRot):(gamepad1.right_stick_x);
      }
      
    //AutonomousMovements
      Fn_MoveAuto();
      
    //Processing
      Fn_MoveProcessing();
      
  }
  
  private void Fn_MoveAuto() {
    //Autonomous Movement Code
    
    //inputs
      TargetLockGOAL = (gamepad1.xWasPressed())?(!TargetLockGOAL):(TargetLockGOAL);
      
      if (TargetLockGOAL) {}
  }
  
  private void Fn_MoveProcessing() {
    //Movement Processing Code
      
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
      ItkMP = -gamepad1.right_stick_y;
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
      /*telemetry.addData("▲", gamepad1.dpad_up ? 1 : 0);
      telemetry.addData("▼", gamepad1.dpad_down ? 1 : 0);
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
      telemetry.addData("Field-Centric", (fieldCentric));
      telemetry.addData("Target Lock onto GOAL", TargetLockGOAL);
      telemetry.addData("Target Lock GOAL color", ((TargetLockGOAL_Target)?("20 ─ RED"):("24 ─ Blue")));
      
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