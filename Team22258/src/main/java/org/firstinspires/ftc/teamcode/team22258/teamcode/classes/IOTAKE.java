package org.firstinspires.ftc.teamcode.team22258.teamcode.classes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**Version 2.0.1*/
@Configurable
public class IOTAKE {
  // Intake/Outtake Class
  
  // Flywheel Definitions TODO: Make Outtake firing RPM distance-based.
  public enum FlywheelState {
    OFF,
    MIN,
    MID,
    MAX,
    BIG
  }
  public static double
    OFF_RPM =    0,
    MIN_RPM = 3000,
    MAX_RPM = 3200;
  
  boolean flywheelDirection = true; // TODO: Make Flywheel Direction Enum-based.
  private FlywheelState flywheelState = FlywheelState .OFF ;
  
  public final int secondsPerMinute
    = 60;
  
  public static double TicksPerRevolutionOfFlywheel
    = 28;
  
  /** Ticks Per Second / Revolutions Per Minute **/
  public double TPSPerRPM() {
    return TicksPerRevolutionOfFlywheel / secondsPerMinute;
    
  }
  
  private DcMotorEx FlywheelMotor;
  double targetFlywheelVelocity;
  
  // VMAX tps left 2620
  // VMAX tps right 2620
  // Feedforward = 32767 / 2620 = 12.5
  public static double
    flywheelP =   20,
    flywheelI =    0,
    flywheelD =    0,
    flywheelF = 12.5;
  
  // Launch Gate Definitions
  private Servo GateServo;
  public enum OuttakeServoState {
    OPEN,
    CLOSED
  }
  OuttakeServoState GateServoState
    = OuttakeServoState .CLOSED ;
  
  // Intake Definitions
  private DcMotor IntakeMotor;
  double ItkMP
    = 0;
  
  // Primary Functions
  public void Init(HardwareMap hardwareMap)                   {
    // Initialization Code
      initIntakeMotor(hardwareMap);
      initFlywheel(hardwareMap);
      initGate(hardwareMap);
  }
  public void Run(Gamepad gamepad)                            {
    //Intake/Outtake Code
      runIntake   (gamepad);
      runGate     (gamepad);
      runFlywheel (gamepad);
  }
  public void doTelemetry(TelemetryManager panelsTelemetry)   {
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
      (GateServoState == OuttakeServoState.OPEN )? ("Open:"):
        ("Closed:"))
    );
    panelsTelemetry.debug("Target Outtake RPM: ",(
      (flywheelState == FlywheelState .MIN )? ( "Min" ):
        (flywheelState == FlywheelState .MID )? ( "Mid" ):
          (flywheelState == FlywheelState .MAX )? ( "Max" ):
            ( "OFF" ))
    );
    panelsTelemetry.debug("Current Outtake RPM",
      FlywheelMotor.getVelocity() / TPSPerRPM()
    );
    
    panelsTelemetry.addLine("");
    
  }
  
  // Secondary Functions
  private void initIntakeMotor(HardwareMap hardwareMap)       {
    // Initialize Intake Motor
    
    // Map Hardware
      IntakeMotor = hardwareMap.get( DcMotor .class, "IntakeMotor");
    
    // Set Motor Behaviors
      IntakeMotor.setDirection( DcMotor.Direction .FORWARD );
  }
  private void initFlywheel(HardwareMap hardwareMap)          {
    // Initialize Flywheel Motor
    
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
  private void initGate(HardwareMap hardwareMap)              {
    // Initialize Gate Servo
      GateServo = hardwareMap.get( Servo .class,"GateServo");
  }
  public void runIntake(Gamepad gamepad)                      {
    setIntakeMotorPower(gamepad.right_trigger - gamepad.left_trigger);
  }
  public void runGate(Gamepad gamepad)                        {
    if (gamepad.rightBumperWasPressed()) {
      switch (GateServoState) {
        case OPEN   : GateServoState = (OuttakeServoState .CLOSED ); break;
        case CLOSED : GateServoState = (OuttakeServoState .OPEN   ); break;
      }
    }
    setGatePosition(GateServoState);
  }
  public void runFlywheel(Gamepad gamepad)                    {
    if (gamepad.x || gamepad.a) { flywheelState = FlywheelState .OFF ;}
    if (gamepad.b)              { flywheelState = FlywheelState .MIN ;}
    if (gamepad.y)              { flywheelState = FlywheelState .MAX ;}
    if (gamepad.leftBumperWasPressed()) flywheelDirection = !flywheelDirection;
    setFlywheelState(flywheelState);
  }
  
  // Variable Functions
  public void setTargetFlywheelVelocity(double speed)         {
    targetFlywheelVelocity = speed;
    
  }
  public void setIntakeMotorPower(double power)               {
    ItkMP = power;
    IntakeMotor.setPower( power );
  }
  public void setGatePosition(OuttakeServoState inputtedState){
    GateServoState = inputtedState;
    GateServo.setPosition(
      (inputtedState == OuttakeServoState.OPEN )?0:1
    );
  }
  public void setFlywheelState(FlywheelState flywheelState)   {
    //Code to run Flywheel motors.
    
    // Switch Velocity
    switch(flywheelState) {
      case OFF: setTargetFlywheelVelocity(OFF_RPM); break;
      case MIN: setTargetFlywheelVelocity(MIN_RPM); break;
      case MAX: setTargetFlywheelVelocity(MAX_RPM); break;
    }
    setTargetFlywheelVelocity(targetFlywheelVelocity * -(flywheelDirection?1:-1));
    
    // Set Velocity
    FlywheelMotor.setVelocity( targetFlywheelVelocity * TPSPerRPM() );
    
  }
  
}