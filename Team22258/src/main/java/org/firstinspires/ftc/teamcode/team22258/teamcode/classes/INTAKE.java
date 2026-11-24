package org .firstinspires .ftc .teamcode .team22258 .teamcode .classes;

import com .bylazar  .configurables .annotations .Configurable    ;
import com .bylazar  .telemetry                  .TelemetryManager;
import com .qualcomm .robotcore     .hardware    .DcMotor         ;
import com .qualcomm .robotcore     .hardware    .HardwareMap     ;

@Configurable
public class INTAKE {
  // Intake Code
  
  // Definitions
    private DcMotor IntakeMotor;
    double ItkMP = 0;
    
  // Functions
    public void   Init                ( HardwareMap hardwareMap          )  {
      // Initialize Intake
        IntakeMotor = hardwareMap.get( DcMotor .class, "IntakeMotor");
        IntakeMotor.setDirection( DcMotor.Direction .FORWARD );
        
    }
    public void   Run                 ( double power                     )  {
      // Set Intake Motor Power
        ItkMP = power;
        IntakeMotor.setPower( power );
        
    }
    public double getIntakeMotorPower (                                  )  {
      // Get Intake Motor Power
        return ItkMP;
        
    }
    public void   doTelemetry         ( TelemetryManager panelsTelemetry )  {
      // Preform Telemetry for Intake
        panelsTelemetry.debug(
          ( "Intake Power |" ),
          ( ItkMP )
        );
        panelsTelemetry.addLine("");
        
    }
    
}