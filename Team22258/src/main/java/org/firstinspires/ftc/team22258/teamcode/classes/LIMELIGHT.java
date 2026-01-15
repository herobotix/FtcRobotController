package org.firstinspires.ftc.team22258.teamcode.classes;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class LIMELIGHT {
  
  public enum TargetLock {
    NONE,
    RED,
    BLUE
  }
  
  //private LockTarget lockTarget;
  private TargetLock targetLock = TargetLock.NONE;
  
  private Limelight3A limelight;
  private IMU rIMU;
  
  double powTurn;
  
  public TargetLock getTargetLock() {
    return targetLock;
  }
  
  public void setTargetLock(TargetLock iState) {
    targetLock = iState;
  }
  
  public double getPowTurn() {
    return powTurn;
  }
  
  public void setPowTurn(double powTurn) {
    this.powTurn = powTurn;
  }
  
  public void Init(HardwareMap hardwareMap) {
    
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
      setTargetLock(
        (targetLock == TargetLock.NONE)? (TargetLock.BLUE):
        (targetLock == TargetLock.BLUE)? (TargetLock.RED):
        (TargetLock.NONE)
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
            ( getTargetLock() != TargetLock.NONE ) &&
            ( aprilTagID == (
              (targetLock == TargetLock.BLUE)? (20):
              (24)
            ))
          ) {
            powTurn = aprilTagXRot * (20);
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
            (getTargetLock() != LIMELIGHT.TargetLock.NONE)? ("Focus"):
              ("Field")
          ):
            (getTargetLock() != LIMELIGHT.TargetLock.NONE)? ("Target"):
              ("Robot")
        ) + "-Centric"
      )
    );
    telemetry.addData(
      (
        "Selected Target"
      ),
      (
        (getTargetLock() == LIMELIGHT.TargetLock.BLUE)? ("20 ─ Blue"):
          (getTargetLock() == LIMELIGHT.TargetLock.RED)? ("24 ─ RED"):
            ("NONE")
      )
    );
  }
  
  public void Stop() {
    limelight.stop();
  }
  
}
