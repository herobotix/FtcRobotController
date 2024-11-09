package org.firstinspires.ftc.schaub.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
import java.util.ArrayList;
import java.util.Comparator;

public class VisionProcessor {
    private Limelight3A limelight = null;
    private IMU imu = null;

    public VisionProcessor(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);

            /*
             * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
             */
            limelight.start();

            // IMU is utilized to maximize accuracy for bot pose calculations
            imu = hardwareMap.get(IMU.class, "imu");
        }
        catch (IllegalArgumentException exception)
        {
            limelight = null;
        }

    }

    public List<VisionTarget> run(Telemetry telemetry) {
        List<VisionTarget> targets = new ArrayList<>();

        if (limelight != null) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            // For maximum 3D localization accuracy, call updateRobotOrientation() and use
            // getBotPose_MT2(). MegaTag2 is an IMU-Fused robot localizer that utilizes the imu to solve
            // the ambiguity problem which is fundamental to all planar targets such as AprilTags.
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                //telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

                    telemetry.addData("Botpose", botpose.toString());

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                        // trigonometry to estimate distance.
                        double cameraHeightMeters = .254; // approximately 10 inches
                        double verticalAngleRadians = Math.toRadians(cr.getTargetYDegrees());
                        double distanceToTargetMeters = cameraHeightMeters / Math.tan(verticalAngleRadians);

                        targets.add(new VisionTarget(cr.getTargetXDegrees(), distanceToTargetMeters));
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }
        }
        else
        {
            telemetry.addData("Limelight", "Not Detected");
        }
        // Sort the list by distance, ensuring the closest target is first
        targets.sort(Comparator.comparingDouble(VisionTarget::getDistance));

        return targets;
    }

    public void stop() {
        limelight.stop();
    }

}
