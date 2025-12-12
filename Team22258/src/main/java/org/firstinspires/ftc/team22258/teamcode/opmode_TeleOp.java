package org.firstinspires.ftc.team22258.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "Opmode (TeleOp) [1.2.4]")
public class opmode_TeleOp extends LinearOpMode {
  
  double Rot;
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
  
  private DcMotor ItkMotor;
  double ItkMP;
  
  private DcMotor OtkMotor;
  private Servo OtkServo;
  double OtkMP;
  byte b_ThrottleMode = 0;
  boolean OtkSs = true;
  
  private LIMELIGHT Limelight;
  
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
      rIMU = hardwareMap.get(IMU.class, "rIMU");
      
    // Set Robot Orientation (IMU)
      IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
      );
      rIMU.initialize(parameters);
      
    // Set Robot Rotation Value
      Rot = rIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
      targetRot = Rot;
      
    // Set Motor Behaviors
      FLMotor.setDirection(DcMotor.Direction.REVERSE);
      FRMotor.setDirection(DcMotor.Direction.FORWARD);
      BLMotor.setDirection(DcMotor.Direction.FORWARD);
      BRMotor.setDirection(DcMotor.Direction.FORWARD);
      ItkMotor.setDirection(DcMotor.Direction.REVERSE);
      OtkMotor.setDirection(DcMotor.Direction.REVERSE);
      OtkMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    
      Limelight = new LIMELIGHT();
      Limelight.Init(hardwareMap);
    
  }
  
  private void Fn_OnStart() {
    //Run On START
    
    //Limelight
      Limelight.Start();
      
  }
  
  private void Fn_Move() {
    //Movement Code
    
    // Get/Reset Robot Rotation Value
      if (gamepad1.start) {rIMU.resetYaw();}
      Rot = rIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
      
    // Inputs
      fieldCentric = gamepad1.bWasPressed() == (!fieldCentric); // Field-Centrism Toggle
      if (fieldCentric) {
        powHead = gamepad1.left_stick_x * Math.sin(Rot) + -gamepad1.left_stick_y * Math.cos(Rot);
        powSide = gamepad1.left_stick_x * Math.cos(Rot) + -gamepad1.left_stick_y * Math.sin(Rot);
        
        if (!Limelight.isTargetLockGOAL()) {
          //Calculate Target Rotation
            targetRot = targetRot + gamepad1.right_stick_x;
            targetRot = (
              (targetRot > Math.PI)?  (targetRot - (2 * Math.PI)):
              (targetRot < -Math.PI)? (targetRot + (2 * Math.PI)):
              (targetRot)
            );
            
          //Calculate Turn Power
            powTurn = (targetRot - Rot) / (2*Math.PI);
            powTurn = ( // FIX THIS
              (powTurn > Math.PI)?  (powTurn - (2 * Math.PI)):
              (powTurn < -Math.PI)? (powTurn + (2 * Math.PI)):
              (powTurn)
            );
            powTurn = (
              (Math.abs(powTurn - (2 * Math.PI)) < Math.PI)?  (powTurn - (2 * Math.PI)):
              (powTurn < -Math.PI)? (powTurn + (2 * Math.PI)):
              (powTurn)
            );
            
        } else {
          powTurn = Limelight.getTargetLockXRot();
        }
        
      } else {
        powHead = -gamepad1.left_stick_y;
        powSide = gamepad1.left_stick_x;
        powTurn = (
          Limelight.isTargetLockGOAL()? (Limelight.getTargetLockXRot()):
          (gamepad1.right_stick_x)
        );
      }
      
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
  
  private void Fn_IOtk() {
    //Intake/Outtake Code
    
    //Intake
      ItkMP = gamepad2.right_trigger;
      ItkMotor.setPower(ItkMP);
      
    //Outtake Motor
     /*Vars*/ double k0 = 0.67, k1 = 0.76, k2 = 0.93;
      b_ThrottleMode = //Toggle Throttle Mode
        (byte) (
          ((gamepad2.x)? ( 0 ):
          ((gamepad2.y)? ( 1 ):
          ((gamepad2.b)? ( 2 ):
          ((gamepad2.a)? ( 3 ):
          (b_ThrottleMode)
        )))))
      ;
      OtkMP = //OtkMP Calc
        (
          ((b_ThrottleMode == 0)? ( 0 ):  //Stopped
          ((b_ThrottleMode == 1)? ( k0 ): //Long Range
          ((b_ThrottleMode == 3)? ( k1 ): //Medium Range
          ( k2 )                          //Short Range
        ))))
      ;
      OtkMotor.setPower(OtkMP); //Set Outtake Motor Power
      
    //Outtake Servo
      OtkSs = gamepad2.rightBumperWasPressed() == (!OtkSs);
      OtkServo.setPosition(OtkSs?(1):(0));
      
  }
  
  private void Fn_Telemetry() {
    //Telemetry Data
      
    //Movement
      telemetry.addLine("Movement ─");
      telemetry.addData("Head Power",powHead);
      telemetry.addData("Side Power",powSide);
      telemetry.addData("Turn Power",powTurn);
      telemetry.addData("Target Rot",targetRot);
      telemetry.addData("Current Rot",Rot);
      telemetry.addLine();
      
    //IO-take
      telemetry.addLine("IOtk ─");
      telemetry.addData(
        ( "Otk | " +
          (
            (OtkServo.getPosition()==1)? ("▲ |"):
            ("▼ |")
          )
        ),
        (
          (
            (b_ThrottleMode == 0)? ( "Short" ):
            (b_ThrottleMode == 1)? ( "Medium" ):
            (b_ThrottleMode == 2)? ( "Long" ):
            ( "None" )
          ) + "\"Range\""
        )
      );
      telemetry.addLine("\"No Intake Connected\"");
      telemetry.addLine();
      
    //Misc
      telemetry.addLine("Miscellaneous ─");
      telemetry.addData(
        (
          "Centricity"
        ),
        (
          (
            fieldCentric? (
              Limelight.isTargetLockGOAL()? ("Focus"):
              ("Field")
            ):
            Limelight.isTargetLockGOAL()? ("Target"):
            ("Robot")
          ) + "-Centric"
        )
      );
      telemetry.addData(
        (
          "Selected Target"
        ),
        (
          (Limelight.getTargetLockColor() == LIMELIGHT.TargetLockColor.BLUE)? ("20 ─ Blue"):
          ("24 ─ RED")
        )
      );
      
    //End Code
      telemetry.update();
      
  }
  
  private void Fn_LoopEnd() {
    //Loop End Code
      /* Zero Values */
        FLMP =
        FRMP =
        BLMP =
        BRMP =
        ItkMP =
        OtkMP =
        (0)
      ;
      Limelight.setTargetLockXRot(0);
  }
  
  private void Fn_OnStop() {
    //Run On STOP
    
    //Limelight
    Limelight.Stop();
    
  }
  
}