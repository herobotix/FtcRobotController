package org.firstinspires.ftc.team22258.teamcode.classes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class IOTAKE {
  
  // Definitions
  
    public enum FlywheelState {
      OFF,
      MIN,
      MID,
      MAX
    }
    public static double
      OFF = 0.00,
      MIN = 0.67,
      MID = 0.76,
      MAX = 0.93
    ;
    
    boolean flywheelDirection = true;
    private FlywheelState flywheelState = FlywheelState.OFF;
    
    private DcMotor ItkMotor;
    double ItkMP;
    
    private DcMotor OtkMotor;
    double flywheelPower;
    
    
    
    private Servo LServo;
    private Servo RServo;
    
    private enum ServoState {
      OPEN,
      CLOSED
    }
    ServoState OtkSs = ServoState.CLOSED;
  
  public void Init(HardwareMap hardwareMap) {
    // Initialization Code
    
    // Map Hardware
      ItkMotor = hardwareMap.get(DcMotor.class, "ItkMotor");
      OtkMotor = hardwareMap.get(DcMotor.class, "OtkMotor");
      LServo = hardwareMap.get(Servo.class, "LServo");
      RServo = hardwareMap.get(Servo.class, "RServo");
      
    // Set Motor Behaviors
      ItkMotor.setDirection(DcMotor.Direction.REVERSE);
      OtkMotor.setDirection(DcMotor.Direction.REVERSE);
      OtkMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      
  }
  
  public void setFlywheelPower(double speed) { flywheelPower = speed; }
  
  public void setIntakeMotorPower(double power) { ItkMP = power; }
  
  public void IOtk(Gamepad gamepad) {
    //Intake/Outtake Code
    
    //Intake
      setIntakeMotorPower( gamepad.left_trigger - gamepad.right_trigger );
    
    //Outtake Motor
      if (gamepad.x) { flywheelState = FlywheelState .OFF ;}
      if (gamepad.a) { flywheelState = FlywheelState .MIN ;}
      if (gamepad.b) { flywheelState = FlywheelState .MID ;}
      if (gamepad.y) { flywheelState = FlywheelState .MAX ;}
      if (gamepad.leftBumperWasPressed()) flywheelDirection = !flywheelDirection;
      switch(flywheelState) {
        case OFF: flywheelPower = OFF; break;
        case MIN: flywheelPower = MIN; break;
        case MID: flywheelPower = MID; break;
        case MAX: flywheelPower = MAX; break;
      }
      flywheelPower = flywheelPower * (flywheelDirection?1:-1);
    
    // Set Flywheel Power
      OtkMotor.setPower(flywheelPower);
      
    // Set Intake Power
      ItkMotor.setPower(ItkMP);
      
    //Outtake Servo
      OtkSs =
        (gamepad.rightBumperWasPressed())?
          ((OtkSs == ServoState .OPEN )? (ServoState .CLOSED ):
          (ServoState .OPEN )
        ):
        (OtkSs)
      ;
      setOtkServoPos();
    
  }
  
  private void setOtkServoPos() {
    LServo.setPosition(
      (OtkSs == ServoState.OPEN)?1:0
    );
    RServo.setPosition(
      (OtkSs == ServoState.OPEN)?0:1
    );
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
        ( "Otk | " +
          (
            (OtkSs == ServoState.OPEN)? ("Open |"):
              ("Closed |")
          )
        ),
        (
          (
            (flywheelState == FlywheelState.MIN)? ( "Short Range" ):
              (flywheelState == FlywheelState.MID)? ( "Medium Range" ):
                (flywheelState == FlywheelState.MAX)? ( "Long Range" ):
                  ( "OFF" )
          )
        )
      );
      
      telemetry.addLine();
  }
  
}
