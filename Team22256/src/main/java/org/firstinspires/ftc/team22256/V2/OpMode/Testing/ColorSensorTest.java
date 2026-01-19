
package org.firstinspires.ftc.team22256.V2.OpMode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


@TeleOp

public class ColorSensorTest extends LinearOpMode {

NormalizedColorSensor sensor;
public enum COLOR_DETECTED{
    RED,
    GREEN,
    BLUE,
    UNKNOWN;
}

    @Override
    public void runOpMode() {

        sensor = hardwareMap.get(NormalizedColorSensor.class,"color1");


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            float red = colors.red;
            float green = colors.green;
            float blue = colors.blue;
            float alpha = colors.alpha;

            float normRed = red / alpha;
            float normBlue = blue / alpha;
            float normGreen = green / alpha;




            telemetry.addData("RED",red);
            telemetry.addData("GREEN",green);
            telemetry.addData("BLUE",blue);
            telemetry.addData("ALPHA",alpha);
            telemetry.addData("NORM RED",normRed);
            telemetry.addData("NORM GREEN",normGreen);
            telemetry.addData("NORM BLUE",normBlue);
            telemetry.update();

        }
    }
}