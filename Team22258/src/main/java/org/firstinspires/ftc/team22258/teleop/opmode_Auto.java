package org.firstinspires.ftc.team22258.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Disabled
@Autonomous(name = "Opmode (Auto) [1.2.0]")
public class opmode_Auto extends LinearOpMode {

  private DcMotor Arm;
  private DcMotor FLM;
  private DcMotor FRM;
  private DcMotor BLM;
  private DcMotor BRM;
  private Servo LClaw;
  private Servo RClaw;

  public final double Pi = 3.1415926535;
  public final double sqrt8 = 2.8284271247;
  
  public final double wD = 2.99 /* +/-0.004 */;
  public final double pR = 10.5;
  
  public final double TicksPerRevolution = (28 * 2.89 * 5.23);
  public final double TicksPerInch = (TicksPerRevolution/(Pi*wD));
  
  public final double RotCorrection = 1;
  public final double RotPerRevolution = ((wD)/(pR*sqrt8)) * RotCorrection;
  public final double TicksPerRot = TicksPerRevolution/RotPerRevolution;
  
  double FLMP;
  double FRMP;
  double BLMP;
  double BRMP;
  double MP;
  double MD;
  
  double FLMD;
  double FRMD;
  double BLMD;
  double BRMD;
  
  double Lynn;
  double Lake;
  double Rick;
  double FArmInput;
  int Clawn = 0;
  double Naptime;
  
  double MPN;

  private IMU imu;
  
  @Override
  public void runOpMode() {
    Arm = hardwareMap.get(DcMotor.class, "Arm");
    FLM = hardwareMap.get(DcMotor.class, "FLMotor");
    FRM = hardwareMap.get(DcMotor.class, "FRMotor");
    BLM = hardwareMap.get(DcMotor.class, "BLMotor");
    BRM = hardwareMap.get(DcMotor.class, "BRMotor");
    LClaw = hardwareMap.get(Servo.class, "LClaw");
    RClaw = hardwareMap.get(Servo.class, "RClaw");

      Fn_IMU();
      Fn_Startup();
      waitForStart();
    if (opModeIsActive()) {
      Fn_Run();
    }
  }

/**
   * Describe this function...
   */
  private void Fn_IMU() {
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
  private void Fn_Startup() {
    //Stop, Resest, & Run using Motor Encoders
      FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    //Stop Arm w/ input of 0
      Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }
  
  private void Fn_Head(
    double dstHead, double strHead
  ) {
  //Back & Forth Movement
    
    //Reset Encoders
      FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    // Drive Power & Distance
      MP = (strHead < 1)?strHead:1;
      MD = (int)(dstHead * TicksPerInch);
    
    //telemetry
      telemetry.addData("Motor Strength", MP);
      telemetry.addData("Motor Distance", MD);
      telemetry.addData("Head Strength", strHead);
      telemetry.addData("Head Distance", dstHead);
      telemetry.update();
    
    // Set target position
      //FLM.setTargetPosition(MD);
      //FRM.setTargetPosition(MD);
      //BLM.setTargetPosition(MD);
      //BRM.setTargetPosition(MD);
    
    // Set Motors to "BUSY" until target distance reached.
      FLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    //Set Motor Power
      FLM.setPower(FLMP);
      FRM.setPower(FRMP);
      BLM.setPower(BLMP);
      BRM.setPower(BRMP);
    
    while (opModeIsActive()&&(FLM.isBusy()||FRM.isBusy()||BLM.isBusy()||BRM.isBusy())) {
      if (!FLM.isBusy()&&!(FLM.getPower()==0)) {FLM.setPower(0);}
      if (!FRM.isBusy()&&!(FRM.getPower()==0)) {FRM.setPower(0);}
      if (!BLM.isBusy()&&!(BLM.getPower()==0)) {BLM.setPower(0);}
      if (!BRM.isBusy()&&!(BRM.getPower()==0)) {BRM.setPower(0);}
      //telemetry
        telemetry.addData("FLM position", FLM.getCurrentPosition());
        telemetry.addData("FRM position", FRM.getCurrentPosition());
        telemetry.addData("BLM position", BLM.getCurrentPosition());
        telemetry.addData("BRM position", BRM.getCurrentPosition());
        telemetry.addData("FLM On?", FLM.isBusy());
        telemetry.addData("FRM On?", FRM.isBusy());
        telemetry.addData("BLM On?", BLM.isBusy());
        telemetry.addData("BRM On?", BRM.isBusy());
        telemetry.addData("Motor Strength", MP);
        telemetry.addData("Motor Distance", MD);
        telemetry.addData("Head Strength", strHead);
        telemetry.addData("Head Distance", dstHead);
        telemetry.update();
      
    }
    
    // Stop Motors not caught in while loop.
      FLM.setPower(0);
      FRM.setPower(0);
      BLM.setPower(0);
      BRM.setPower(0);
    
    // Return motors to using the encoder.
      FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  
  /* private void Fn_Move(
    double dstHead, double strHead,
    double dstSide, double strSide,
    double dstTurn, double strTurn
  ) {
  //--- Movement
    
    //Reset Encoders
      FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    // Drive & Strafe Power
      FLMP = strHead + strSide - strTurn;
      FRMP = strHead - strSide + strTurn;
      BLMP = strHead - strSide - strTurn;
      BRMP = strHead + strSide + strTurn;
    
    //telemetry
      double FLMPo = FLMP;
      double FRMPo = FRMP;
      double BLMPo = BLMP;
      double BRMPo = BRMP;
    
    // Power Control
      MPN = (
        1/Math.max( 
          Math.max( Math.abs(FLMP), Math.abs(FRMP) ), 
          Math.max( Math.abs(BLMP), Math.abs(BRMP) ) 
        )
      );
      if (MPN < 1){
        BRMP *= MPN;
        BLMP *= MPN;
        FRMP *= MPN;
        FLMP *= MPN;
      }
    
    // Drive & Strafe Distance
      FLMD = (dstHead + dstSide) * TicksPerInch - dstTurn * TicksPerRot;
      FRMD = (dstHead - dstSide) * TicksPerInch + dstTurn * TicksPerRot;
      BLMD = (dstHead - dstSide) * TicksPerInch - dstTurn * TicksPerRot;
      BRMD = (dstHead + dstSide) * TicksPerInch + dstTurn * TicksPerRot;
    
    //telemetry
      telemetry.addData("MPN", MPN);
      telemetry.addData("FLMP", FLMP);
      telemetry.addData("FRMP", FRMP);
      telemetry.addData("BLMP", BLMP);
      telemetry.addData("BRMP", BRMP);
      telemetry.addData("FLMD", FLMD);
      telemetry.addData("FLMD", FRMD);
      telemetry.addData("BLMD", BLMD);
      telemetry.addData("BRMD", BRMD);
      telemetry.addData("FLMP OLD", FLMPo);
      telemetry.addData("FRMP OLD", FRMPo);
      telemetry.addData("BLMP OLD", BLMPo);
      telemetry.addData("BRMP OLD", BRMPo);
      telemetry.update();
    
    // Set target position
      FLM.setTargetPosition((int)FLMD);
      FRM.setTargetPosition((int)FRMD);
      BLM.setTargetPosition((int)BLMD);
      BRM.setTargetPosition((int)BRMD);
    
    // Set Motors to "BUSY" until target distance reached.
      FLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    //Set Motor Power
      FLM.setPower(-FLMP);
      FRM.setPower(-FRMP);
      BLM.setPower(-BLMP);
      BRM.setPower(-BRMP);
    
    while (opModeIsActive()&&(FLM.isBusy()||FRM.isBusy()||BLM.isBusy()||BRM.isBusy())) {
        if (!FLM.isBusy()&&!(FLM.getPower()==0)) {FLM.setPower(0);}
        if (!FRM.isBusy()&&!(FRM.getPower()==0)) {FRM.setPower(0);}
        if (!BLM.isBusy()&&!(BLM.getPower()==0)) {BLM.setPower(0);}
        if (!BRM.isBusy()&&!(BRM.getPower()==0)) {BRM.setPower(0);}
        //telemetry
          telemetry.addData("FLMTp", FLM.getCurrentPosition());
          telemetry.addData("FRMTp", FRM.getCurrentPosition());
          telemetry.addData("BLMTp", BLM.getCurrentPosition());
          telemetry.addData("BRMTp", BRM.getCurrentPosition());
          telemetry.addData("MPN", MPN);
          telemetry.addData("FLMb", FLM.isBusy());
          telemetry.addData("FRMb", FRM.isBusy());
          telemetry.addData("BLMb", BLM.isBusy());
          telemetry.addData("BRMb", BRM.isBusy());
          telemetry.addData("FLMP", FLMP);
          telemetry.addData("FRMP", FRMP);
          telemetry.addData("BLMP", BLMP);
          telemetry.addData("BRMP", BRMP);
          telemetry.addData("FLMD", FLMD);
          telemetry.addData("FRMD", FRMD);
          telemetry.addData("BLMD", BLMD);
          telemetry.addData("BRMD", BRMD);
          telemetry.addData("FLMDt", (int)FLMD);
          telemetry.addData("FRMDt", (int)FRMD);
          telemetry.addData("BLMDt", (int)BLMD);
          telemetry.addData("BRMDt", (int)BRMD);
          telemetry.addData("FLMP OLD", FLMPo);
          telemetry.addData("FRMP OLD", FRMPo);
          telemetry.addData("BLMP OLD", BLMPo);
          telemetry.addData("BRMP OLD", BRMPo);
          telemetry.update();
        
      }
    
    // Stop Motors not caught in while loop.
      FLM.setPower(0);
      FRM.setPower(0);
      BLM.setPower(0);
      BRM.setPower(0);
    
    // Return motors to using the encoder.
      FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  } */
  
  /**
   * Describe this function...
   */
  private void Fn_Clawrm(int Clawn) {
    LClaw.setPosition(0 == Clawn ? 0.75 : 0.25);
    RClaw.setPosition(1 == Clawn ? 0.75 : 0.25);
    
    //telemetry
      telemetry.addData("Clawn", Clawn);
      telemetry.addData("LClawPos", LClaw.getPosition());
      telemetry.addData("RClawPos", RClaw.getPosition());
      telemetry.update();
    
  }

  /**
   * Describe this function...
   */
  private void Fn_NewArmon(double FArmInput) {
    Arm.setPower(FArmInput);
    
    //telemetry
      telemetry.addData("FArmInput", FArmInput);
      telemetry.update();
    
  }

  /**
   * Describe this function...
   */
  private void Fn_Pause(double Naptime) {
      sleep((long)(Naptime*1000));
  }
  
   /* //selection
    Fn_Move(
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0
    );
    Fn_NewArmon(0.0);
    Fn_Clawrm(0);
    Fn_Pause(0.0);
    
   /**
    * Describe this function...
    */
  private void Fn_Run() {
    Fn_Head(
      12.0, 0.5/*,
      6.0, 0.5,
      0.0, 0.0
    );
    Fn_Move(
      0.0, 0.0,
      0.0, 0.0,
      1.0, 0.5*/
    );
  }
}