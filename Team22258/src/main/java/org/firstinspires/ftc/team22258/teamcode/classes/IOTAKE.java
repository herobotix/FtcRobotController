package org.firstinspires.ftc.team22258.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IOTAKE {
  
  
  private DcMotor ItkMotor;
  double ItkMP;
  
  private DcMotor OtkMotor;
  double flywheelSpeed;
  
  private enum FlywheelDirection {
    FORWARDS,
    BACKWARDS
  }
  FlywheelDirection flywheelDirection = FlywheelDirection.FORWARDS;
  
  public enum FlywheelState {
    NONE,
    SHORT,
    MEDIUM,
    LONG
  }
  double NONE = 0.00, SHORT = 0.67, MEDIUM = 0.76, LONG = 0.93;
  
  private FlywheelState flywheelState;
  
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
  
  public void setFlywheelSpeed(double speed) {
    flywheelSpeed = speed;
  }
  public void setIntakeMotorPower(double power) {
    ItkMP = power;
  }
  
  public void IOtk(Gamepad gamepad2) {
    //Intake/Outtake Code
    
    //Intake
      ItkMP = gamepad2.left_trigger - gamepad2.right_trigger;
      ItkMotor.setPower(ItkMP);
    
    //Outtake Motor
      flywheelState = ( //Toggle Throttle Mode
        ((gamepad2.x)? ( FlywheelState.NONE ):
          ((gamepad2.a)? ( FlywheelState.SHORT ):
            ((gamepad2.b)? ( FlywheelState.MEDIUM ):
              ((gamepad2.y)? ( FlywheelState.LONG ):
                (flywheelState)
              )
            )
          )
        )
      );
      switch (flywheelDirection) {
        case FORWARDS:
          flywheelDirection = FlywheelDirection.BACKWARDS;
          break;
        case BACKWARDS:
          flywheelDirection = FlywheelDirection.FORWARDS;
          break;
      }
      switch(flywheelState) {
        case NONE:
          flywheelSpeed = NONE;
          break;
        case SHORT:
          flywheelSpeed = SHORT;
          break;
        case MEDIUM:
          flywheelSpeed = MEDIUM;
          break;
        case LONG:
          flywheelSpeed = LONG;
          break;
      }
      flywheelSpeed = flywheelSpeed * ((flywheelDirection == FlywheelDirection.FORWARDS)?(1):(-1));
    
    //Set Flywheel Power
      OtkMotor.setPower(flywheelSpeed);
      
    //Outtake Servo
      OtkSs = (gamepad2.rightBumperWasPressed())?((OtkSs== ServoState.OPEN)? ServoState.CLOSED: ServoState.OPEN):OtkSs;
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
            (flywheelState == FlywheelState.SHORT)? ( "Short Range" ):
              (flywheelState == FlywheelState.MEDIUM)? ( "Medium Range" ):
                (flywheelState == FlywheelState.LONG)? ( "Long Range" ):
                  ( "OFF" )
          )
        )
      );
      
      telemetry.addLine();
  }
  
}
