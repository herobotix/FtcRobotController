package org.firstinspires.ftc.teamcode.team22258.teamcode.classes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**Version 2.0.2*/
@Configurable
public class OTKFLYWHEEL {
  // Intake/Outtake Class
  
  // Flywheel Definitions TODO: Make Outtake firing RPM distance-based.
  public enum FlywheelState {
    OFF,
    MIN,
    MAX
  }
  public static double
    OFF_RPM =    0,
    MIN_RPM = 2750,
    MAX_RPM = 3200;
  
  boolean flywheelDirection = true; // TODO: Make Flywheel Direction Enum-based.
  double targetFlywheelVelocity;
  private FlywheelState flywheelState = FlywheelState.OFF ;
  
  // TPS Per RPM Definitions
    public final int secondsPerMinute = 60;
    public static double TicksPerRevolutionOfFlywheel = 28;
    public double TPSPerRPM() {
      //Ticks Per Second / Revolutions Per Minute
        return TicksPerRevolutionOfFlywheel / secondsPerMinute;
        
    }
    
  // Flywheel Initialization Definitions
    private DcMotorEx FlywheelMotor;
    public static double
      flywheelP =   20,
      flywheelI =    0,
      flywheelD =    0,
      flywheelF = 12.5; // Feedforward = 32767 / 2620 = 12.5 // VMAX tps 2620
      
  // Primary Functions
    public void Init(HardwareMap hardwareMap)                   {
      // Initialize Flywheel
      
      // Map Hardware
      FlywheelMotor = hardwareMap.get( DcMotorEx .class,"FlywheelMotor");
      
      // Set Motor Behaviors
      FlywheelMotor.setDirection( DcMotor.Direction .FORWARD );
      FlywheelMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior .FLOAT );
      
      // Stop, Reset, & Run using Motor Encoders
      FlywheelMotor.setMode( DcMotor.RunMode .STOP_AND_RESET_ENCODER );
      PIDFCoefficients flywheelPIDF = new PIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF);
      FlywheelMotor.setPIDFCoefficients(DcMotor.RunMode .RUN_USING_ENCODER, flywheelPIDF);
      FlywheelMotor.setMode( DcMotor.RunMode .RUN_USING_ENCODER );
      FlywheelMotor.setVelocity(0);
      
    }
    public void Run(Gamepad gamepad)                            {
      // Run Outtake Flywheel
             if (gamepad.x || gamepad.a)  { setFlywheelState(FlywheelState .OFF ) ;}
        else if (gamepad.b)               { setFlywheelState(FlywheelState .MIN ) ;}
        else if (gamepad.y)               { setFlywheelState(FlywheelState .MAX ) ;}
        if (gamepad.leftBumperWasPressed()) flywheelDirection = !flywheelDirection;
        
        
    }
    public void setFlywheelState(FlywheelState flywheelState)   {
      // Set the Flywheel's State
        switch(flywheelState) {
          case OFF: setFlywheelVelocity(OFF_RPM); break;
          case MIN: setFlywheelVelocity(MIN_RPM); break;
          case MAX: setFlywheelVelocity(MAX_RPM); break;
        }
        
    }
    public void doTelemetry(TelemetryManager panelsTelemetry)   {
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
    
  // Variable Functions
    private void setFlywheelVelocity(double speed)         {
      // Set Velocity of the Flywheel
      targetFlywheelVelocity = speed * (flywheelDirection?1:-1);
      FlywheelMotor.setVelocity( targetFlywheelVelocity * TPSPerRPM() );
      
    }
    
}