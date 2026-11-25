package org.firstinspires.ftc.teamcode.team22258.teamcode.classes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class OTKGATE {
  // Intake/Outtake Class
  
  // Definitions
    private Servo GateServo;
    public enum GatePosition {
      OPEN,
      CLOSED
    }
    GatePosition gatePosition = GatePosition .CLOSED;
    
  // Functions
    public void         Init            ( HardwareMap hardwareMap             ) {
      // Initialize Gate Servo
        GateServo = hardwareMap.get( Servo .class,"GateServo");
        
    }
    public void         doTelemetry     ( TelemetryManager panelsTelemetry    ) {
      // Preform Telemetry for Otkgate
        panelsTelemetry.debug("Outtake Gate:", (
          (getGatePosition() == GatePosition.OPEN )? ("Open"):
            ("Closed"))
        );
        panelsTelemetry.addLine("");
        
    }
    public void         setGatePosition ( GatePosition inputtedState          ) {
      // Set the State of the Outtake Gate
        gatePosition = inputtedState;
        GateServo.setPosition(
          (inputtedState == GatePosition.OPEN )?1:0
        );
        
    }
    public GatePosition getGatePosition (                                     ) {
      // Get Intake Motor Power
        return gatePosition;
        
    }
    
}