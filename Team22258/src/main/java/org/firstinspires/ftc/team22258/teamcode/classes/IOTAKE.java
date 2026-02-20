package org.firstinspires.ftc.team22258.teamcode.classes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**Version 1.2.21*/
@Configurable
public class IOTAKE {
  // Intake/Outtake Class
  
  // Flywheel Definitions
    public enum FlywheelState {
      OFF,
      MIN,
      MID,
      MAX,
      BIG
    }
    public static double
      OFF_RPM =    0,
      MIN_RPM = 3450,
      MID_RPM = 3900,
      MAX_RPM = 4470,
      BIG_RPM = 4470;
    
    boolean flywheelDirection = true;
    private FlywheelState flywheelState = FlywheelState .OFF ;
  
    public final int secondsPerMinute
      = 60;
    
    public static double TicksPerRevolutionOfFlywheel
      = 28;
    
    /** Ticks Per Second / Revolutions Per Minute **/
    public double TPSPerRPM() {
      return TicksPerRevolutionOfFlywheel / secondsPerMinute;
      
    }
    
    private DcMotorEx lFlywheelMotor;
    private DcMotorEx rFlywheelMotor;
    double targetFlywheelVelocity;

    // VMAX tps left 2620
    // VMAX tps right 2620
    // Feedforward = 32767 / 2620 = 12.5
    public static double
      flywheelP =  20,
      flywheelI =  0,
      flywheelD =  0,
      flywheelF = 12.5;
    
  // Launch Gate Definitions
    private Servo lServo;
    private Servo rServo;
    public enum OuttakeServoState {
      OPEN,
      CLOSED
    }
    OuttakeServoState outtakeServoState
      = OuttakeServoState .CLOSED ;
    
  // Intake Definitions
    private DcMotor ItkMotor;
    double ItkMP
      = 0;
    
  // Primary Functions
    public void Init(HardwareMap hardwareMap)                   {
      // Initialization Code
      
      // Map Hardware
        ItkMotor = hardwareMap.get(DcMotor.class, "ItkMotor");
        lFlywheelMotor = hardwareMap.get(DcMotorEx.class, "lFlywheelMotor");
        rFlywheelMotor = hardwareMap.get(DcMotorEx.class, "rFlywheelMotor");
        lServo = hardwareMap.get(Servo.class, "lServo");
        rServo = hardwareMap.get(Servo.class, "rServo");
        
      // Set Motor Behaviors
        ItkMotor.setDirection( DcMotor.Direction .FORWARD );
        lFlywheelMotor.setDirection( DcMotor.Direction .FORWARD );
        lFlywheelMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior .FLOAT );
        rFlywheelMotor.setDirection( DcMotor.Direction .REVERSE );
        rFlywheelMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior .FLOAT );
        
      // Stop, Reset, & Run using Motor Encoders
        lFlywheelMotor.setMode( DcMotor.RunMode .STOP_AND_RESET_ENCODER );
        rFlywheelMotor.setMode( DcMotor.RunMode .STOP_AND_RESET_ENCODER );
        PIDFCoefficients flywheelPIDF = new PIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF);
        lFlywheelMotor.setPIDFCoefficients(DcMotor.RunMode .RUN_USING_ENCODER, flywheelPIDF);
        lFlywheelMotor.setMode( DcMotor.RunMode .RUN_USING_ENCODER );
        lFlywheelMotor.setVelocity(0);
        rFlywheelMotor.setPIDFCoefficients(DcMotor.RunMode .RUN_USING_ENCODER, flywheelPIDF);
        rFlywheelMotor.setMode( DcMotor.RunMode .RUN_USING_ENCODER );
        rFlywheelMotor.setVelocity(0);
    }
    public void Run(Gamepad gamepad)                            {
      //Intake/Outtake Code
      
      // Intake
        runIntake(gamepad.right_trigger - gamepad.left_trigger);
        
      // Outtake Gate
        if (gamepad.rightBumperWasPressed()) {
          switch (outtakeServoState) {
            case OPEN: outtakeServoState = (OuttakeServoState.CLOSED ); break;
            case CLOSED: outtakeServoState = (OuttakeServoState.OPEN ); break;
          }
        }
        runOtkGate(outtakeServoState);
        
      // Outtake Flywheel
        if (gamepad.x) { flywheelState = FlywheelState .OFF ;}
        if (gamepad.a) { flywheelState = FlywheelState .MIN ;}
        if (gamepad.b) { flywheelState = FlywheelState .MID ;}
        if (gamepad.y) { flywheelState = FlywheelState .MAX ;}
        if (gamepad.leftBumperWasPressed()) flywheelDirection = !flywheelDirection;
        runFlywheel(flywheelState);
        
    }
    public void doTelemetry(TelemetryManager panelsTelemetry)  {
      //Telemetry Code
      
      // IO-take
        panelsTelemetry.addLine("IOtk ─");
        
      // Intake
        panelsTelemetry.debug(
          ( "Itk |" ),
          ( ItkMP )
        );
        
      // Outtake
        panelsTelemetry.debug("Outtake Gate:", (
          (outtakeServoState == OuttakeServoState.OPEN )? ("Open:"):
                                                        ("Closed:"))
        );
        panelsTelemetry.debug("Target Outtake RPM: ",(
          (flywheelState == FlywheelState .MIN )? ( "Min" ):
          (flywheelState == FlywheelState .MID )? ( "Mid" ):
          (flywheelState == FlywheelState .MAX )? ( "Max" ):
                                                  ( "OFF" ))
        );
        panelsTelemetry.debug("Current Outtake RPM",
          rFlywheelMotor.getVelocity() / TPSPerRPM()
        );
    
        panelsTelemetry.addLine("");
        
    }
    
  // Secondary Functions
    public void runIntake(double power)                         {
      setIntakeMotorPower( power );
      ItkMotor  .setPower( power );
    }
    public void runFlywheel(FlywheelState flywheelState)        {
      //Code to run Flywheel motors.
      
      // Switch Velocity
        switch(flywheelState) {
          case OFF: targetFlywheelVelocity = OFF_RPM; break;
          case MIN: targetFlywheelVelocity = MIN_RPM; break;
          case MID: targetFlywheelVelocity = MID_RPM; break;
          case MAX: targetFlywheelVelocity = MAX_RPM; break;
          case BIG: targetFlywheelVelocity = BIG_RPM; break;
        }
        targetFlywheelVelocity = targetFlywheelVelocity * -(flywheelDirection?1:-1);
        
      // Set Velocity
        lFlywheelMotor.setVelocity( targetFlywheelVelocity * TPSPerRPM() );
        rFlywheelMotor.setVelocity( targetFlywheelVelocity * TPSPerRPM() );
        
    }
    
  // Variable Functions
    public void setTargetFlywheelVelocity(double speed)         {
      targetFlywheelVelocity = speed;
      
    }
    public void setIntakeMotorPower(double power)               {
      ItkMP = power;
      
    }
    public void runOtkGate(OuttakeServoState outtakeServoState) {
      lServo.setPosition(
        (outtakeServoState == OuttakeServoState.OPEN )?0:1
      );
      rServo.setPosition(
        (outtakeServoState == OuttakeServoState.OPEN )?1:0
      );
    }
    
}