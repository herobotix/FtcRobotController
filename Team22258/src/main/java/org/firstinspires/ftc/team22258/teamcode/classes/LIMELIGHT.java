package org.firstinspires.ftc.team22258.teamcode.classes;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@Configurable
public class LIMELIGHT {
  
  // Definitions
    public enum TargetLock {
      OFF,
      RED,
      BLUE
    }
    
    private TargetLock targetLock = TargetLock .OFF ;
    
    private Limelight3A limelight;
    private IMU rIMU;
    
    double powTurn;
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
            (TargetLock.OFF)
          );
        }
      
      YawPitchRollAngles orientation = rIMU.getRobotYawPitchRollAngles();
      limelight.updateRobotOrientation(orientation.getYaw());
      LLResult results = limelight.getLatestResult();
      telemetry.addLine("Limelight ─");
      if (results != null && results.isValid()) {
        List<LLResultTypes.FiducialResult> fiducials = results.getFiducialResults();
        if (!fiducials.isEmpty()) {
          int index = 0;
          for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int aprilTagID = fiducial.getFiducialId();
            double aprilTagXRot = fiducial.getTargetXDegrees()/(360);
            telemetry.addData("Detection #" + index + " ID:", aprilTagID);
            telemetry.addData("TargetXRot", aprilTagXRot);
            if (
              ( targetLock != TargetLock .OFF ) &&
              ( aprilTagID == (
                (targetLock == TargetLock .BLUE )? (20):
                (24)
              ))
            ) {
              powTurn = aprilTagXRot * powTurnMod;
              telemetry.addData("Target Rotation", powTurn);
            } else {
              powTurn = 0;
            }
            index++;
          }
        } else {
          telemetry.addLine("Limelight Fiducials Empty");
          telemetry.addLine();
        }
      } else {
        telemetry.addLine("No AprilTags Detected");
        telemetry.addLine();
      }
      
    }
    
    public void doTelemetry(Telemetry telemetry, boolean fieldCentric) {
      
      telemetry.addLine("Miscellaneous ─");
      telemetry.addData(
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
      telemetry.addData(
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
