
package org.firstinspires.ftc.team22256.teleop;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Configurable
@TeleOp(name = "Sensor: Limelight3A", group = "Sensor")
public class LimeLightTest extends LinearOpMode {

    private Limelight3A limelight;
    private DcMotor turret;
    private PIDFController Controller0;
    private double  error= 0;
    private double currentTurretPos = 0;
    private final double TICKS_PER_DEGREE = (double) 116 /180;
    private double turretOutput = 0;
    private static double p = 0.035;
    private static double i  =0;
    private static double d = 0.001;
    private static double f = 0;


    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = hardwareMap.get(DcMotor.class, "turret");
        Controller0 = new PIDFController(p,i,d,f);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();


        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());


            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
            } else {
                telemetry.addData("Limelight", "No data available");

            }

            currentTurretPos = turret.getCurrentPosition();
            error = result.getTx() * TICKS_PER_DEGREE;//Distance from limelight to AprilTag in degrees
            double target =  (currentTurretPos + error);



            turretOutput = Controller0.calculate(currentTurretPos,target);//Use PID to calculate output
          // turretOutput = Controller0.calculate(result.getTx(),0);
            turret.setPower(turretOutput);

            telemetry.addData("target",target);
            telemetry.addData("output",turretOutput);
            telemetry.addData("error",error);
            telemetry.addData("current position",currentTurretPos);
            telemetry.addData("turret power",turret.getPower());
            telemetry.update();




        }
        limelight.stop();
    }
}
