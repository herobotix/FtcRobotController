package org.firstinspires.ftc.team22258.teamcode.classes;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**Version 1.2.20*/
@Configurable
public class LIMELIGHT {
  // Limelight Class
  
  // Hardware Definitions
    private Limelight3A limelight;
    private IMU rIMU;
    
  // Target Lock Definitions
    public enum TargetLock {
      OFF(-1),
      RED(24),
      BLUE(20);
      
      private final int value;
      TargetLock(int value){
        this.value = value;
      }
      
      public int getValue() {
        return value;
      }
      
    }
    private TargetLock targetLock = TargetLock .OFF ;

    public static double targetBearingDegrees = 0.0;

  // Turn Power Definitions
    double powTurn = 0.0;
    public static double powTurnMod = 15;
    
  // Main Functions
    
    public void Init(HardwareMap hardwareMap) {
      //Map Hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        rIMU = hardwareMap.get(IMU.class, "rIMU");
        
    }
    
    public void Start() {
      //Initialize Limelight
        limelight.pipelineSwitch(0);
        limelight.start();
        
    }
    
    public void Run(Gamepad gamepad1, Telemetry telemetry) {
      //Limelight Code
      
      //LockOn Detection
        if (gamepad1.xWasPressed()) {
          targetLock = (
            (targetLock == TargetLock .OFF )? ( TargetLock .BLUE ):
            (targetLock == TargetLock .BLUE )? ( TargetLock .RED ):
            (TargetLock .OFF )
          );
        }
      
      double robotYawDegrees = rIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit .DEGREES );
      limelight.updateRobotOrientation( robotYawDegrees );
      
      telemetry.addLine("Limelight ─");

      boolean found = false;

      LLResult results = limelight.getLatestResult();
      if (results != null && results.isValid() ) {
        
        List<LLResultTypes.FiducialResult> fiducials = results.getFiducialResults();
        
        if (!fiducials.isEmpty() ) {
          
          for (LLResultTypes.FiducialResult fiducial : fiducials) {
            
            int aprilTagID = fiducial.getFiducialId();
            telemetry.addData("ID:", aprilTagID);
            
            if (aprilTagID == targetLock.getValue() ) {
              targetBearingDegrees = fiducial.getTargetXDegrees();

              // We want to align slight left/right of fiducial to ensure we
              // hit the highest corner edge of the backstop.
              if (aprilTagID == TargetLock.BLUE.getValue())
              {
                targetBearingDegrees -= 3.5;
              }
              if (aprilTagID == TargetLock.RED.getValue())
              {
                targetBearingDegrees += 1.0;
              }

              powTurn = powTurnMod * targetBearingDegrees / 360;
              found = true;
              telemetry.addData("Target Bearing Degrees: ", targetBearingDegrees);
              telemetry.addData("Target Rotation", powTurn);
            }
            
          }
        } else {
          telemetry.addLine("Limelight Fiducials Empty");
          telemetry.addLine();
        }
      } else {
        telemetry.addLine("No AprilTags Detected");
        telemetry.addLine();
      }

      if (!found)
      {
        powTurn = 0.0;
        targetBearingDegrees = 5000.0;
        telemetry.addData("Target Bearing Degrees: ", targetBearingDegrees);
      }
    }
    
    public void doTelemetry(TelemetryManager panelsTelemetry, boolean fieldCentric) {
      
      panelsTelemetry.addLine("Miscellaneous ─");
      panelsTelemetry.debug(
        (
          "Centricity"
        ),
        (
          (
            fieldCentric? (
              (targetLock != TargetLock .OFF )? ("Focus"):
                ("Field")
            ):
              (targetLock != TargetLock .OFF )? ("Target"):
                ("Robot")
          ) + "-Centric"
        )
      );
      panelsTelemetry.debug(
        (
          "Selected Target"
        ),
        (
          (targetLock == TargetLock .BLUE )? ("20 ─ Blue"):
            (targetLock == TargetLock .RED )? ("24 ─ RED"):
              ("OFF")
        )
      );
    }
    
    public void Stop() { limelight.stop(); }
    
  // Variable Functions
    
    public TargetLock getTargetLock() { return targetLock; }
    
    // public void setTargetLock(TargetLock iState) { targetLock = iState;}
    
    public double getPowTurn() { return powTurn; }
    
    public void setPowTurn(double powTurn) { this.powTurn = powTurn; }
    
}
