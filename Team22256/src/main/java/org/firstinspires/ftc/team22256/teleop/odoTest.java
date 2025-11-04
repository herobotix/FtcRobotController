
package org.firstinspires.ftc.team22256.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.team22256.common.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp
public class odoTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    GoBildaPinpointDriver odo;
    @Override
    public void runOpMode() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setOffsets(-140.0, -175.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        odo.recalibrateIMU();
        odo.resetPosAndIMU();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.update();

        }
    }
}