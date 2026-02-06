package org.firstinspires.ftc.team22258.teamcode.classes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class IOTAKE {
  // Intake/Outtake Class
  
  // Flywheel Definitions
    public enum FlywheelState {
      OFF,
      MIN,
      MID,
      MAX
    }
    public static double OFF_RPM = 0, MIN_RPM = 2400, MID_RPM = 2600, MAX_RPM = 2900;
    
    boolean flywheelDirection = true;
    private FlywheelState flywheelState = FlywheelState .OFF ;
  
    public static double TicksPerRevolution = 28; //Ticks per Revolution of the Wheel
  
    public final int secondsPerMinute = 60;
    
    private DcMotorEx OtkMotor;
    double targetFlywheelVelocity;
  
    public static double flywheelP = 1.15, flywheelI = 0.05, flywheelD = 0.0, flywheelF = 11.8;
    public static PIDFCoefficients testPIDF = new PIDFCoefficients(1.15, 0.05, 0.0, 11.8);
    
  // Launch Gate Definitions
    private Servo LServo;
    private Servo RServo;
    
    private enum ServoState {
      OPEN,
      CLOSED
    }
    ServoState OtkSs = ServoState .CLOSED ;
    
  // Intake Definitions
    private DcMotor ItkMotor;
    double ItkMP;
    
  // Main Functions
    
    public void Init(HardwareMap hardwareMap) {
      // Initialization Code
      
      // Map Hardware
        ItkMotor = hardwareMap.get(DcMotor.class, "ItkMotor");
        OtkMotor = hardwareMap.get(DcMotorEx.class, "OtkMotor");
        LServo = hardwareMap.get(Servo.class, "LServo");
        RServo = hardwareMap.get(Servo.class, "RServo");
        
      // Set Motor Behaviors
        ItkMotor.setDirection( DcMotor.Direction .REVERSE );
        OtkMotor.setDirection( DcMotor.Direction .REVERSE );
        OtkMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior .FLOAT );
        
      // Stop, Reset, & Run using Motor Encoders
        OtkMotor.setMode( DcMotor.RunMode .STOP_AND_RESET_ENCODER );
        PIDFCoefficients flywheelPIDF = new PIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF);
        OtkMotor.setPIDFCoefficients(DcMotor.RunMode .RUN_USING_ENCODER, flywheelPIDF);
        OtkMotor.setMode( DcMotor.RunMode .RUN_USING_ENCODER );
        
    }
    
    public void Run(Gamepad gamepad) {
      //Intake/Outtake Code
      
      //Intake
      
      // Set Intake Power
        setIntakeMotorPower( gamepad.left_trigger - gamepad.right_trigger );
        ItkMotor.setPower(ItkMP);
        
      
      //Outtake Servo
        OtkSs = (gamepad.rightBumperWasPressed())?
          ((OtkSs == ServoState .OPEN )? (ServoState .CLOSED ):
            (ServoState .OPEN )
          ):
          (OtkSs)
        ;
        setOtkServoPos();
        
      
      //Outtake
      
      // Velocity Switcher
        if (gamepad.x) { flywheelState = FlywheelState .OFF ;}
        if (gamepad.a) { flywheelState = FlywheelState .MIN ;}
        if (gamepad.b) { flywheelState = FlywheelState .MID ;}
        if (gamepad.y) { flywheelState = FlywheelState .MAX ;}
        if (gamepad.leftBumperWasPressed()) flywheelDirection = !flywheelDirection;
        switch(flywheelState) {
          case OFF: targetFlywheelVelocity = OFF_RPM; break;
          case MIN: targetFlywheelVelocity = MIN_RPM; break;
          case MID: targetFlywheelVelocity = MID_RPM; break;
          case MAX: targetFlywheelVelocity = MAX_RPM; break;
        }
        targetFlywheelVelocity = targetFlywheelVelocity * -(flywheelDirection?1:-1);
        
      // Reset Encoders
        OtkMotor.setMode(DcMotor.RunMode .STOP_AND_RESET_ENCODER );
        
      // Reset Run Mode
        OtkMotor.setMode(DcMotor.RunMode .RUN_USING_ENCODER );
        PIDFCoefficients flywheelPIDF = new PIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF);
        OtkMotor.setPIDFCoefficients(DcMotor.RunMode .RUN_USING_ENCODER , flywheelPIDF);
      
      // Set Velocity
        OtkMotor.setVelocity( targetFlywheelVelocity * TicksPerRevolution / secondsPerMinute );
        
    }
    
    public void doTelemetry(Telemetry telemetry) {
    // IO-take
      telemetry.addLine("IOtk ─");
      
    // Intake
      telemetry.addData(
        ( "Itk |" ),
        ( ItkMP )
      );
      
    // Outtake
      telemetry.addData(
        ( "Outtake Gate:"
        ),
        (
          (
            (OtkSs == ServoState .OPEN )? ("Open:"):
              ("Closed:")
          )
        )
      );
      telemetry.addData(
        ( "Target Outtake RPM: "
        ),
        (
          (
            (flywheelState == FlywheelState .MIN )? ( "Min" ):
              (flywheelState == FlywheelState .MID )? ( "Mid" ):
                (flywheelState == FlywheelState .MAX )? ( "Max" ):
                  ( "OFF" )
          )
        )
      );
      telemetry.addData("Current Outtake RPM", OtkMotor.getVelocity() * secondsPerMinute / TicksPerRevolution );
      
      telemetry.addLine();
  }
  
  // Variable Functions
    public void setTargetFlywheelVelocity(double speed) { targetFlywheelVelocity = speed; }
    
    public void setIntakeMotorPower(double power) { ItkMP = power; }
    
    private void setOtkServoPos() {
    LServo.setPosition(
      (OtkSs == ServoState .OPEN )?1:0
    );
    RServo.setPosition(
      (OtkSs == ServoState .OPEN )?0:1
    );
  }
  
}
