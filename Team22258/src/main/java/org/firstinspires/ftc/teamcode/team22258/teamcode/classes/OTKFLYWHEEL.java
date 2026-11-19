package org.firstinspires.ftc.teamcode.team22258.teamcode.classes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
public class OTKFLYWHEEL {
  // Intake/Outtake Class
  
  // Definitions
    private DcMotorEx FlywheelMotor;
    public static double // TODO: Make Outtake firing RPM distance-based.
      OFF_RPM =    0,
      MIN_RPM = 2750,
      MAX_RPM = 3200;
    public enum FlywheelState                                                   {
      OFF,
      MIN,
      MAX
    }
    public enum FlywheelDirection                                               {
      FORWARDS(+1),
      REVERSED(-1);
      final double value;
      FlywheelDirection(double value) {
        this.value=value;
      }
    }
    private FlywheelState flywheelState = FlywheelState .OFF ;
    FlywheelDirection flywheelDirection = FlywheelDirection .FORWARDS ;
    
  // Functions
    public  void   Init                    ( HardwareMap hardwareMap          ) {
      // Initialize Flywheel
      
      // Map Hardware
        FlywheelMotor = hardwareMap.get( DcMotorEx .class,"FlywheelMotor");
        
      // Set Motor Behaviors
        FlywheelMotor.setDirection         ( DcMotor.Direction         .FORWARD                );
        FlywheelMotor.setZeroPowerBehavior ( DcMotor.ZeroPowerBehavior .FLOAT                  );
        FlywheelMotor.setMode              ( DcMotor.RunMode           .STOP_AND_RESET_ENCODER );
        
      // flywheel PIDF Coefficients
        final double
          flywheelP =   20,
          flywheelI =    0,
          flywheelD =    0,
          flywheelF = 12.5; // Feedforward = 32767 / 2620 = 12.5 // VMAX tps 2620
        PIDFCoefficients flywheelPIDF = new PIDFCoefficients(
          flywheelP, flywheelI, flywheelD, flywheelF
        );
        FlywheelMotor.setPIDFCoefficients  ( DcMotor.RunMode   .RUN_USING_ENCODER, flywheelPIDF);
        FlywheelMotor.setMode              ( DcMotor.RunMode   .RUN_USING_ENCODER               );
        FlywheelMotor.setVelocity(0);
        
    }
    public  void   Run                     ( Gamepad gamepad                  ) {
      // Run Outtake Flywheel
             if (gamepad.x || gamepad.a)  { setFlywheelState(FlywheelState .OFF ) ;}
        else if (gamepad.b)               { setFlywheelState(FlywheelState .MIN ) ;}
        else if (gamepad.y)               { setFlywheelState(FlywheelState .MAX ) ;}
        if (gamepad.leftBumperWasPressed()) {
          switch (flywheelDirection) {
            case FORWARDS: flywheelDirection = FlywheelDirection .REVERSED; break;
            case REVERSED: flywheelDirection = FlywheelDirection .FORWARDS; break;
          }
        }
        
        
    }
    public  void   setFlywheelState        ( FlywheelState inputflywheelState ) {
      // Set the Flywheel's State
        switch(flywheelState = inputflywheelState) {
          case OFF: setFlywheelVelocity(OFF_RPM); break;
          case MIN: setFlywheelVelocity(MIN_RPM); break;
          case MAX: setFlywheelVelocity(MAX_RPM); break;
        }
        
    }
    private void   setFlywheelVelocity     ( double speed                     ) {
      // Set Velocity of the Flywheel
        FlywheelMotor.setVelocity( speed * flywheelDirection.value * TPSPerRPM() );
        
    }
    public  void   doTelemetry             ( TelemetryManager panelsTelemetry ) {
      //Telemetry Code
      
      // Outtake
      panelsTelemetry.debug("Target Outtake RPM: ",(
        (flywheelState == FlywheelState.MIN )? ( "Min" ):
        (flywheelState == FlywheelState.MAX )? ( "Max" ):
        ( "OFF" )
      ));
      panelsTelemetry.debug("Current Outtake RPM",
        FlywheelMotor.getVelocity() / TPSPerRPM()
      );
      
      panelsTelemetry.addLine("");
      
    }
    public  double TPSPerRPM               (                                  ) {
    //Ticks Per Second / Revolutions Per Minute
    final int secondsPerMinute = 60;
    final double TicksPerRevolutionOfFlywheel = 28;
    return TicksPerRevolutionOfFlywheel / secondsPerMinute;
    
  }
    public  FlywheelState getFlywheelState (                                  ) {
    // Get Outtake Flywheel State
    return flywheelState;
  }
  
}