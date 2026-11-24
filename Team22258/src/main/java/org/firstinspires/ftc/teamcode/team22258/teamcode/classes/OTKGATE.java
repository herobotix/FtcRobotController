package org.firstinspires.ftc.teamcode.team22258.teamcode.classes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class OTKGATE {
  // Intake/Outtake Class
  
  // Definitions
    private Servo GateServo;
    public enum OuttakeGateServoState                                   {
      OPEN,
      CLOSED
    }
    OuttakeGateServoState GateServoState = (
      OuttakeGateServoState .CLOSED
    );
    
  // Functions
    public void Init            ( HardwareMap hardwareMap             ) {
      // Initialize Gate Servo
        GateServo = hardwareMap.get( Servo .class,"GateServo");
        
    }
    public void Run             ( Gamepad gamepad                     ) {
      //Intake/Outtake Code
        if (gamepad.rightBumperWasPressed()) {
          switch (GateServoState) {
            case OPEN   : GateServoState = (OuttakeGateServoState.CLOSED ); break;
            case CLOSED : GateServoState = (OuttakeGateServoState.OPEN   ); break;
          }
        }
        setGatePosition(GateServoState);
        
    }
    public void doTelemetry     ( TelemetryManager panelsTelemetry    ) {
      // Preform Telemetry for Otkgate
        panelsTelemetry.debug("Outtake Gate:", (
          (GateServoState == OuttakeGateServoState.OPEN )? ("Open"):
            ("Closed"))
        );
        panelsTelemetry.addLine("");
        
    }
    public void setGatePosition ( OuttakeGateServoState inputtedState ) {
      // Set the State of the Outtake Gate
        GateServoState = inputtedState;
        GateServo.setPosition(
          (inputtedState == OuttakeGateServoState.OPEN )?1:0
        );
        
    }
    
}