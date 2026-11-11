package org.firstinspires.ftc.teamcode.team22258.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.DRIVE;
import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.IOTAKE;
import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.RIMU;

import com.bylazar.configurables.annotations.Configurable;

@TeleOp(name = "Opmode (TeleOp)")
@Configurable
public class opmode_TeleOp extends LinearOpMode {
  //Driver-Controlled Opmode
  
    
  // Driving Definitions
    boolean fieldCentric
      = false;
    double
      powHead,
      powSide,
      powTurn;
    
  // Indicator Light Definition
    //private Servo indicatorLight;
    
  // Class Definitions
    private RIMU      rIMU  ;
    private DRIVE     Drive ;
    //private LIMELIGHT Limelight;
    private IOTAKE    IOtake;
  
  // Telemetry Definition
    private TelemetryManager panelsTelemetry;
    
  // Run Function
    @Override
    public void runOpMode()     {
      //Begin
      
      // Init & Wait
        OnInit();
        waitForStart();
        OnStart();
        
      // Run Opmode
        while (opModeIsActive()) { OnLoop(); }
        OnStop();
        
    }
    
  // Main Functions
    private void OnLoop()       {
      //Looped Code
        //Limelight.Run(gamepad1, telemetry);
        Fn_Inputs   (                      );
        Drive .Run  ( rIMU.getYawRadians() );
        IOtake.Run  ( gamepad2             );
        doTelemetry (                      );
        LoopEnd     (                      );
  }
    private void OnInit()       {
      // Initialization Code
      
      // Telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        
      // Map Hardware
        //indicatorLight = hardwareMap.get( Servo   .class, "indicatorLight");
        
      // Classes
        rIMU = new RIMU()        ;
        rIMU.Init( hardwareMap );
        Drive     = new DRIVE()       ;
        Drive     .Init( hardwareMap );
        //Limelight = new  LIMELIGHT()  ;
        //Limelight .Init( hardwareMap );
        IOtake    = new  IOTAKE()     ;
        IOtake    .Init( hardwareMap );
        
    }
    private void OnStart()      {
      //Run On START
      
      //Limelight
        //Limelight.Start();
        
    }
    private void OnStop()       {
      //Run On STOP
      
      //Limelight
        //Limelight.Stop();
        
    }
  
  // In-Loop Functions
    private void Fn_Inputs()    {
    // Inputs Code
    
    // Reset Robot Rotation Value
      if (gamepad1.start) {rIMU.resetYaw();}
      
    // Inputs
      fieldCentric = gamepad1.bWasPressed() == (!fieldCentric); // Field-Centrism Toggle
      powHead = -gamepad1.left_stick_y;
      powSide = gamepad1.left_stick_x;
      powTurn = (
        /*(Limelight.getTargetLock() != LIMELIGHT.TargetLock .OFF )? (Limelight.getPowTurn()):*/
        (gamepad1.right_stick_x)
      );
      
    // GoBilda RGB indicator light, PWM values from GoBilda product page
      /*if(indicatorLight != null) {
          if (Limelight.getTargetLock() == LIMELIGHT.TargetLock.OFF) {
              indicatorLight.setPosition(0.0);
          } else if (Limelight.getTargetLock() == LIMELIGHT.TargetLock.BLUE) {
              indicatorLight.setPosition(0.611);
          } else if (Limelight.getTargetLock() == LIMELIGHT.TargetLock.RED) {
              indicatorLight.setPosition(0.29);
          }
      }*/
  }
    private void doTelemetry()  {
      // Preform Telemetry
      
      // Do Telemetry for Classes
        rIMU.doTelemetry( panelsTelemetry               );
        Drive     .doTelemetry( panelsTelemetry               );
        //Limelight .doTelemetry( panelsTelemetry, fieldCentric );
        IOtake    .doTelemetry( panelsTelemetry               );
        
      // Update Telemetry
        panelsTelemetry.update( telemetry );
        
    }
    private void LoopEnd()      {
      //Loop End Code
      
      // Zero Values
        Drive.LoopEnd();
        //Limelight.setPowTurn(0);
        IOtake.setTargetFlywheelVelocity(0.);
        IOtake.setIntakeMotorPower(0.);
        
    }
    
}