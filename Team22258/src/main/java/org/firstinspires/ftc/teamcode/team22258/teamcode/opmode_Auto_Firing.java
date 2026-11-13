package org.firstinspires.ftc.teamcode.team22258.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.DRIVE;
import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.IOTAKE;
import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.RIMU;

@Autonomous(name = "Opmode (Auto_Firing)")
@Configurable
public class opmode_Auto_Firing extends LinearOpMode {
  //Driver-Controlled Opmode
  
  // Panels' Variables
    public static double
      Head0 = 0., Side0 = .8, Turn0 = 0., Time0 = 1. ,
      Head1 =-1., Side1 = 0., Turn1 = 0., Time1 = 1.3,
      Head2 = 5., Side2 = 5., Turn2 = 0., Time2 = .3 ,
      Head3 =-5., Side3 = 5., Turn3 = 0., Time3 = .5 ;
    
  // Class Definitions
    private RIMU      rIMU  ;
    private DRIVE     Drive ;
    //private LIMELIGHT Limelight;
    private IOTAKE    IOtake;
  
  // Telemetry Definition
    private TelemetryManager panelsTelemetry;
    
  // Run Function
    @Override
    public void runOpMode   (                                                )  {
      //Begin
      
      // Init & Wait
        OnInit();
        waitForStart();
        OnStart();
        
    }
    
  // Main Functions
    private void OnInit     (                                                )  {
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
    private void OnStart    (                                                )  {
      //Run On START
      
      // Do Stuff
      IOtake     .setFlywheelState   ( IOTAKE.FlywheelState      .MIN  );
                                 cont(2.                       );
      IOtake     .setGatePosition    ( IOTAKE.OuttakeServoState  .OPEN );
      IOtake     .setIntakeMotorPower( -0.5                            );
                                 cont(4.                       );
      Move(Head0, Side0, Turn0); cont(Time0);
      Move(Head1, Side1, Turn1); cont(Time1);
      Move(Head2, Side2, Turn2); cont(Time2);
      Move(Head3, Side3, Turn3); cont(Time3);
      
    }
    
  // Secondary Functions
    private void Move       ( double powHead, double powSide, double powTurn )  {
    // Inputs Code
      
    // Inputs
      Drive.powHead = powHead;
      Drive.powSide = powSide;
      Drive.powTurn = powTurn;
      
    // Run Move Code
      Drive.Run(rIMU.getYawRadians());
      
  }
    private void doTelemetry(                                                )  {
      // Preform Telemetry
      
      // Do Telemetry for Classes
        rIMU.doTelemetry( panelsTelemetry               );
        Drive     .doTelemetry( panelsTelemetry               );
        //Limelight .doTelemetry( panelsTelemetry, fieldCentric );
        IOtake    .doTelemetry( panelsTelemetry               );
        
      // Update Telemetry
        panelsTelemetry.update( telemetry );
        
    }
    private void cont       ( double seconds                                 )  {
      //continue with current (time in seconds)
        doTelemetry ();
        long milliseconds = (long) (1000 * seconds);
        sleep( milliseconds );
        doTelemetry ();
        Drive.LoopEnd();
        IOtake.setTargetFlywheelVelocity(0.);
        IOtake.setIntakeMotorPower(0.);
        
    }
    
}