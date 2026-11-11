package org.firstinspires.ftc.teamcode.team22258.teamcode.classes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class DRIVE {
  // Drivetrain & Movement Code
  
  // Definitions
    private DcMotor FLMotor , FRMotor , BLMotor , BRMotor ;
    private double  FLMP    , FRMP    , BLMP    , BRMP    ;
    public  double  powHead , powSide , powTurn ;
    public  boolean fieldCentric = false;
    
  // Functions
    public void Init        (HardwareMap      hardwareMap     ) {
      // Initialization Code
      
      // Map Hardware
        FLMotor        = hardwareMap.get( DcMotor .class, "FLMotor"       );
        FRMotor        = hardwareMap.get( DcMotor .class, "FRMotor"       );
        BLMotor        = hardwareMap.get( DcMotor .class, "BLMotor"       );
        BRMotor        = hardwareMap.get( DcMotor .class, "BRMotor"       );
        
      // Set Motor Behaviors
        FLMotor .setDirection( DcMotor.Direction .REVERSE );
        FRMotor .setDirection( DcMotor.Direction .FORWARD );
        BLMotor .setDirection( DcMotor.Direction .FORWARD );
        BRMotor .setDirection( DcMotor.Direction .FORWARD );
        
    }
    public void Run         (double           robotYawRadians ) {
      //Movement Code
        
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
      double MPN = (
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
    public void doTelemetry (TelemetryManager panelsTelemetry ) {
      // Preform Telemetry for Movement Code
        panelsTelemetry.addLine("Movement ─");
        panelsTelemetry.debug("Head Power", powHead );
        panelsTelemetry.debug("Side Power", powSide );
        panelsTelemetry.debug("Turn Power", powTurn );
        panelsTelemetry.addLine("");
        
    }
    public void LoopEnd     (                                 ) {
      //Loop End Code / Zero Values
        FLMP = (0);
        FRMP = (0);
        BLMP = (0);
        BRMP = (0);
        
    }
    
}