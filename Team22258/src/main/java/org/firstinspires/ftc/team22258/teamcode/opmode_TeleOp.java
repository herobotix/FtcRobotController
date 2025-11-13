package org.firstinspires.ftc.team22258.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;


@TeleOp(name = "Opmode (TeleOp) [1.1.17]")
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
  
  boolean fieldCentric = true;
  double MPN;
  
  double powHead, powSide, powTurn;
  
  private DcMotor ItkMotor;
  double ItkMP;
  
  private DcMotor OtkMotor;
  private Servo OtkServo;
  double OtkMP;
  //byte b_ThrottleType = 3;
  byte b_ThrottleMode = 3;
  boolean OtkSs = true;
  
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
        Fn_Move();
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
      OtkServo = hardwareMap.get(Servo.class, "OtkServo");
      limelight = hardwareMap.get(Limelight3A.class, "limelight");
      rIMU = hardwareMap.get(IMU.class, "rIMU");
      
    // Set Motor Behaviors
      FLMotor.setDirection(DcMotor.Direction.FORWARD);
      FRMotor.setDirection(DcMotor.Direction.REVERSE);
      BLMotor.setDirection(DcMotor.Direction.REVERSE);
      BRMotor.setDirection(DcMotor.Direction.REVERSE);
      ItkMotor.setDirection(DcMotor.Direction.REVERSE);
      OtkMotor.setDirection(DcMotor.Direction.REVERSE);
      OtkMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      
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
  
  private void Fn_Limelight() {
    //Limelight Code
    
    //LockOn Detection
      TargetLockGOAL = gamepad1.xWasPressed() == (!TargetLockGOAL);
      TargetLockGOAL_Target = gamepad1.yWasPressed() == (!TargetLockGOAL_Target);
    
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
            double aprilTagXRot = fiducial.getTargetXDegrees()/(3.6* 4.7 );
            telemetry.addLine("Limelight ─");
            telemetry.addData("Detection #" + index + " ID:", aprilTagID);
            telemetry.addData("TargetXRot", aprilTagXRot);
            if (TargetLockGOAL&&( aprilTagID == ((TargetLockGOAL_Target)?(20):(24)) )) {
              TargetLockXRot = aprilTagXRot;
              telemetry.addData("TargetLockXRot", TargetLockXRot);
            } else {
              TargetLockXRot = 0;
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
  
  private void Fn_Move() {
    //Movement Code
    
    // Get/Reset Robot Rotation Value
      if (gamepad1.start) {rIMU.resetYaw();}
      Rot = rIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
      
    // Field-Centrism Toggle
      fieldCentric = gamepad1.bWasPressed() == (!fieldCentric);
      
    // Inputs
      if (fieldCentric) {
        powHead = gamepad1.left_stick_x * Math.sin(Rot) + gamepad1.left_stick_y * Math.cos(Rot);
        powSide = gamepad1.left_stick_x * Math.cos(Rot) - gamepad1.left_stick_y * Math.sin(Rot);
      } else {
        powHead = gamepad1.left_stick_y;
        powSide = gamepad1.left_stick_x;
      }
      powTurn = TargetLockGOAL?(TargetLockXRot):(gamepad1.right_stick_x);
      
    //Processing
      Fn_MoveProcessing();
      
  }
  
  private void Fn_MoveProcessing() {
    //Movement Processing Code
      
    // Drive & Strafe & Rotate
      FLMP = powHead - powSide - powTurn;
      FRMP = powHead + powSide + powTurn;
      BLMP = powHead + powSide - powTurn;
      BRMP = powHead - powSide + powTurn;
      
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
      ItkMP = gamepad2.right_trigger;
      ItkMotor.setPower(ItkMP);
      
    //Outtake Motor
     /*Vars*/ //double h0 = 0.15, h1 = 0.49, h2 = 0.83, x = -gamepad2.right_stick_y;
              double k0 = 0.76, k1 = 0.87, k2 = 0.93;
      /*b_ThrottleType = (byte) ((gamepad2.xWasPressed() && gamepad2.left_bumper)? //Toggle Throttle Type
        ((b_ThrottleType == 3)?
          (0):
          (b_ThrottleType + 1)
        ):
        (b_ThrottleType)
      );*/
      b_ThrottleMode = (byte) (/*(b_ThrottleType == 1)? //Toggle Throttle Mode
        ( Math.signum(x) * ((Math.abs(x) < h1)?  //Throttle "Steps"
          ( 0 ):               //First Step
          ((Math.abs(x) < h2)?
            ( 1 ):             //Second Step
            ( 2 )              //Third Step
          )
        )):
        ((b_ThrottleType == 3)?*/
          ((gamepad2.yWasPressed())?
            ( 2 ):
            ((gamepad2.bWasPressed())?
              ( 1 ):
              ((gamepad2.aWasPressed())?
                ( 0 ):
                ((gamepad2.xWasPressed())?
                  ( 3 ):
                  (b_ThrottleMode)
                )
              )
            )
          )/*:
          ((gamepad2.yWasPressed())?
            ((b_ThrottleMode == 2)?
              (0):
              (b_ThrottleMode + 1)
            ):
            (b_ThrottleMode)
          )
        )*/
      );
      OtkMP = (/*(b_ThrottleType == 3)?*/ //OtkMP Calc
        ((b_ThrottleMode == 0)?  //Throttle "Ranges"
          ( k0 ):               //Short Range
          ((b_ThrottleMode == 1)?
            ( k1 ):             //Medium Range
            ((b_ThrottleMode == 2)?
              ( k2 ):           //Long Range
              ( 0 )             //Stopped
            )
          )
        )/*:
        ((Math.abs(x) < h0)?
          ( k0 * x / h0):
          ((b_ThrottleType == 0)?
            ((( 1 - k0)*( Math.abs(x) - h0)/( 1 - h0) + k0)*( Math.signum(x) )): //Throttle "Slopes"
            ( Math.signum(x) * ((b_ThrottleMode == 0)?  //Throttle "Steps" & "Modes"
              ( k0 ):               //First Step/Mode
              ((b_ThrottleMode == 1)?
                ( k1 ):             //Second Step/Mode
                ( k2 )              //Third Step/Mode
              )
            ))
          )
        )*/
      );
      OtkMotor.setPower(OtkMP); //Set Outtake Motor Power
      
    //Outtake Servo
      OtkSs = gamepad2.rightStickButtonWasPressed() == (!OtkSs);
      OtkServo.setPosition(OtkSs?(1):(0));
      
  }
  
  private void Fn_Telemetry() {
    //Telemetry Data
      
    //Movement
      telemetry.addLine("Movement ─");
      telemetry.addData("Head Power",powHead);
      telemetry.addData("Side Power",powSide);
      telemetry.addData("Turn Power",powTurn);
      telemetry.addLine();
      
    //IO-take
      telemetry.addLine("IOtk ─");
      telemetry.addData("Otk | " + ((OtkServo.getPosition()==1)?("▲ |"):("▼ |")), OtkMP);
      telemetry.addData("Throttle " + (/*
        ((b_ThrottleType==0)?
          ("Type: \"Slopes\""):
          ((b_ThrottleType==1)?
            ("\"Step\""):
            ((b_ThrottleType==2)?
              ("\"Mode\""):
              (*/"\"Range\""/*)
            )
          )
        )
      */), (
        (/*(b_ThrottleType==0)?
          (""):
          ((b_ThrottleType==3)?
            (*/(b_ThrottleMode == 0)?
              ( "Short" ):
              ((b_ThrottleMode == 1)?
                ( "Medium" ):
                ((b_ThrottleMode == 2)?
                  ( "Long" ):
                  ( "None" )
                )
              )
            )/*:
            (b_ThrottleMode)
          )
        )
      */));
      telemetry.addLine("\"No Intake Connected\"");
      telemetry.addLine();
      
    //Misc
      telemetry.addLine("Miscellaneous ─");
      telemetry.addData("Centricity", (fieldCentric?(TargetLockGOAL?("Target"):("Robot")):(TargetLockGOAL?("Focus"):("Field")))+"-Centric");
      telemetry.addData("Selected Target", ((TargetLockGOAL_Target)?("20 ─ Blue"):("24 ─ RED")));
      
    //End Code
      telemetry.update();
      
  }
  
  private void Fn_LoopEnd() {
    //Loop End Code
      TargetLockXRot = 0;
      FLMP = 0;
      FRMP = 0;
      BLMP = 0;
      BRMP = 0;
      ItkMP = 0;
      OtkMP = 0;
  }
  
  private void Fn_OnStop() {
    //Run On STOP
    
    //Limelight
    limelight.stop();
    
  }
  
}