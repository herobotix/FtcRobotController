package org.firstinspires.ftc.schaub.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "LimelightTest")
public class LimelightTest extends LinearOpMode {

  private Limelight3A limelight = null;
  private IMU imu = null;

  @Override
  public void runOpMode() {
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    limelight.pipelineSwitch(0);

    imu = hardwareMap.get (IMU.class, "imu");
    RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    waitForStart();

    limelight.start();
    //Run Opmode
    while (opModeIsActive()) {
      YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
      limelight.updateRobotOrientation(orientation.getYaw());
      LLResult results = limelight.getLatestResult();
      if (results != null && results.isValid()) {
//        Pose3D botPose = results.getBotpose_MT2();
//
//        telemetry.addData("Tx", results.getTx());
//        telemetry.addData("Ty", results.getTy());
//        telemetry.addData("Ta", results.getTa());

        List<LLResultTypes.FiducialResult> fiducials = results.getFiducialResults();
        if (!fiducials.isEmpty()) {
          int index = 0;
          for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int aprilTagId = fiducial.getFiducialId();
            telemetry.addData("Detection " + index + " ID", aprilTagId);
            index++;
          }

        } else {
          telemetry.addLine("No AprilTags detected");
        }

        telemetry.update();

      }
    }
    limelight.stop();
  }
}