package org.firstinspires.ftc.team22258.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
  
  public enum TargetLockColor {
    RED,
    BLUE
  }
  
  private TargetLockColor targetLockColor;
  
  private Limelight3A limelight;
  private IMU rIMU;
  
  boolean TargetLockGOAL = false;
  boolean TargetLockGOAL_Target = false;
  double TargetLockXRot;
  
  public boolean isTargetLockGOAL() {
    return TargetLockGOAL;
  }
  
  public boolean isTargetLockGOAL_Target() {
    return TargetLockGOAL_Target;
  }
  
  public TargetLockColor getTargetLockColor() {
    return targetLockColor;
  }
  
  public double getTargetLockXRot() {
    return TargetLockXRot;
  }
  
  public void setTargetLockXRot(double targetLockXRot) {
    TargetLockXRot = targetLockXRot;
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
  
  public LIMELIGHT() {
  
  }
  
  public void Run(Gamepad gamepad1, Telemetry telemetry) {
    //Limelight Code
    
    //LockOn Detection
    TargetLockGOAL = gamepad1.xWasPressed() == (!TargetLockGOAL);
    TargetLockGOAL_Target = gamepad1.yWasPressed() == (!TargetLockGOAL_Target);
    
    //
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
          if (TargetLockGOAL&&( aprilTagID == ((TargetLockGOAL_Target)?(20):(24)) )) {
            TargetLockXRot = aprilTagXRot * (20);
            telemetry.addData("TargetLockXRot", TargetLockXRot);
          } else {
            TargetLockXRot = 0;
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
  
  public void Stop() {
    limelight.stop();
  }
  
}
