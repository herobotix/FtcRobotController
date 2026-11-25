package org.firstinspires.ftc.teamcode.team22258.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.DRIVE;
import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.INTAKE;
import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.OTKFLYWHEEL;
import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.OTKGATE;
import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.RIMU;

import com.bylazar.configurables.annotations.Configurable;

@TeleOp(name = "Opmode (TeleOp)")
@Configurable
public class opmode_TeleOp extends LinearOpMode {
  //Driver-Controlled Opmode
  
  // Indicator Light Definition
  /*private Servo indicatorLight;*/
    
  // Definitions
    private TelemetryManager panelsTelemetry ;
    private RIMU             rIMU            ;
    private DRIVE            Drive           ;
    private INTAKE           Intake          ;
    private OTKGATE          Gate            ;
    private OTKFLYWHEEL      Flywheel        ;
  /*private LIMELIGHT        Limelight       ;*/
    
  // Run Function
    @Override
    public void runOpMode    () {
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
    private void OnLoop      () {
      //Looped Code
        Fn_Inputs   ();
        doTelemetry ();
        LoopEnd     ();
  }
    private void OnInit      () {
      // Initialization Code
      
      // Telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        
      // Map Hardware
        //indicatorLight = hardwareMap.get( Servo   .class, "indicatorLight");
        
      // Classes
        rIMU      = new RIMU()        ;
        rIMU      .Init( hardwareMap );
        Drive     = new DRIVE()       ;
        Drive     .Init( hardwareMap );
        Intake     = new INTAKE()     ;
        Intake    .Init( hardwareMap );
        Gate = new OTKGATE()    ;
        Gate.Init( hardwareMap );
        Flywheel  = new OTKFLYWHEEL() ;
        Flywheel  .Init( hardwareMap );
      /*Limelight = new  LIMELIGHT()  ;
        Limelight .Init( hardwareMap );*/
        
    }
    private void OnStart     () {
      //Run On START
      
      //Limelight
        //Limelight.Start();
        
    }
    private void OnStop      () {
      //Run On STOP
      
      //Limelight
        //Limelight.Stop();
        
    }
  
  // In-Loop Functions
    private void Fn_Inputs   () {
      // Inputs Code
      
      // rIMU Inputs
        if (gamepad1.start) {rIMU.resetYaw();} // Reset Robot Rotation Value
        
      // Drive Inputs
        Drive.fieldCentric = gamepad1.bWasPressed() == (!Drive.fieldCentric); // Field-Centrism Toggle
        Drive.Run  (
          (-gamepad1.left_stick_y), //iHead
          (gamepad1.left_stick_x) , //iSide
          /*(Limelight.getTargetLock() != LIMELIGHT.TargetLock .OFF )? (Limelight.getPowTurn()):*/
          (gamepad1.right_stick_x), //iTurn
          (rIMU.getYawRadians())
        );
        
      // Intake Inputs
        Intake.Run(gamepad2.right_trigger - gamepad2.left_trigger);
        
      // Gate Inputs
        if ( gamepad2 .rightBumperWasPressed() ) {
          switch ( Gate.getGatePosition() ) {
            case OPEN   : Gate.setGatePosition( OTKGATE.GatePosition .CLOSED ); break;
            case CLOSED : Gate.setGatePosition( OTKGATE.GatePosition .OPEN   ); break;
          }
        }
        
      // Flywheel Inputs
             if ( gamepad2.x || gamepad2.a    ) { Flywheel.setFlywheelState(OTKFLYWHEEL.FlywheelState.OFF ) ;}
        else if ( gamepad2.b                  ) { Flywheel.setFlywheelState(OTKFLYWHEEL.FlywheelState.MIN ) ;}
        else if ( gamepad2.y                  ) { Flywheel.setFlywheelState(OTKFLYWHEEL.FlywheelState.MAX ) ;}
        if ( gamepad2.leftBumperWasPressed()  ) {
          switch ( Flywheel.flywheelDirection ) {
            case FORWARDS: Flywheel.flywheelDirection = OTKFLYWHEEL.FlywheelDirection.REVERSED; break;
            case REVERSED: Flywheel.flywheelDirection = OTKFLYWHEEL.FlywheelDirection.FORWARDS; break;
          }
        }
        
      // Limelight Inputs
      /*Limelight.Run(gamepad1, telemetry);*/
        
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
    private void doTelemetry () {
      // Preform Telemetry
      
      // Do Telemetry for Classes
        rIMU      .doTelemetry( panelsTelemetry               );
        Drive     .doTelemetry( panelsTelemetry               );
        Intake    .doTelemetry( panelsTelemetry               );
        Gate      .doTelemetry( panelsTelemetry               );
        Flywheel  .doTelemetry( panelsTelemetry               );
      /*Limelight .doTelemetry( panelsTelemetry, fieldCentric );*/
        
      // Update Telemetry
        panelsTelemetry.update( telemetry );
        
    }
    private void LoopEnd     () {
      //Loop End Code
      
      // Zero Values
        Drive.LoopEnd();
        //Limelight.setPowTurn(0);
        Flywheel.setFlywheelState(OTKFLYWHEEL.FlywheelState.OFF);
        Intake  .Run(0.);
        
    }
    
}