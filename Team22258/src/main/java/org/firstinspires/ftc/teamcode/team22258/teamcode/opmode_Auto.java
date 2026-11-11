package org.firstinspires.ftc.teamcode.team22258.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Opmode (Auto) [2.0.1]")
@Configurable
public class opmode_Auto extends LinearOpMode {
  //Robot-Controlled Opmode
  
  // Panels' Variables
    public static double
      Head0 = .5, Side0 = 0., Turn0 = 0., Time0 = .5,
      Head1 = 0., Side1 = 0., Turn1 = 0., Time1 = 0.,
      Head2 = 0., Side2 = 0., Turn2 = 0., Time2 = 0.;
    
  
  // Drivetrain Definitions
    private DcMotor
      FLMotor,
      FRMotor,
      BLMotor,
      BRMotor;
    double
      FLMP,
      FRMP,
      BLMP,
      BRMP;
    
  // Driving Definitions
    boolean fieldCentric
      = false;
    double
          MPN,
      powHead,
      powSide,
      powTurn;
    
  // Position Definitions
    double robotYawRadians;
    private IMU rIMU;
    
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
        
    }
    
  // Main Functions
    private void OnInit()       {
      // Initialization Code
      
      // Telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        
      // Map Hardware
        FLMotor        = hardwareMap.get( DcMotor .class, "FLMotor"       );
        FRMotor        = hardwareMap.get( DcMotor .class, "FRMotor"       );
        BLMotor        = hardwareMap.get( DcMotor .class, "BLMotor"       );
        BRMotor        = hardwareMap.get( DcMotor .class, "BRMotor"       );
        rIMU           = hardwareMap.get( IMU     .class, "rIMU"          );
  
      // Set Robot Orientation (IMU)
        IMU.Parameters parameters =
          new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection .UP,
            RevHubOrientationOnRobot.UsbFacingDirection  .BACKWARD
          )
        );
        rIMU.initialize(parameters);
        
      // Set Robot Rotation Value
        robotYawRadians = rIMU.getRobotYawPitchRollAngles().getYaw( AngleUnit .RADIANS );
        
      // Set Motor Behaviors
        FLMotor .setDirection( DcMotor.Direction .REVERSE );
        FRMotor .setDirection( DcMotor.Direction .FORWARD );
        BLMotor .setDirection( DcMotor.Direction .FORWARD );
        BRMotor .setDirection( DcMotor.Direction .FORWARD );
        
    }
    private void OnStart()      {
      //Run On START
      
      // Do Stuff
        Move(Head0, Side0, Turn0, Time0);
        Move(Head1, Side1, Turn1, Time1);
        Move(Head2, Side2, Turn2, Time2);
    }
  
  // Secondary Functions
    private void Move(double iHead, double iSide, double iTurn, double seconds)  {
      //Movement Code
      
      //Inputs
        powHead = iHead;
        powSide = iSide;
        powTurn = iTurn;
      
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
      
      // wait
        wait(seconds);
      
      // Telemetry
        doTelemetry();
    }
    private void doTelemetry()                                   {
      // Preform Telemetry
      
      // Do Telemetry for Movement Code
        panelsTelemetry.addLine("Movement ─");
        panelsTelemetry.debug("Head Power", powHead );
        panelsTelemetry.debug("Side Power", powSide );
        panelsTelemetry.debug("Turn Power", powTurn );
        panelsTelemetry.debug("Current Robot Yaw",  robotYawRadians / (2*Math.PI) );
        panelsTelemetry.addLine("");
        
      // Update Telemetry
        panelsTelemetry.update( telemetry );
        
    }
    private void wait(double seconds) {
      //wait (time in seconds)
        long milliseconds = (long) (1000 * seconds);
        sleep( milliseconds );
    }
    
}