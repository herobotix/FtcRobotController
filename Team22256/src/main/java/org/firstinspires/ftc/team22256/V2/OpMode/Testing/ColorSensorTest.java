
package org.firstinspires.ftc.team22256.V2.OpMode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


@TeleOp

public class ColorSensorTest extends LinearOpMode {

NormalizedColorSensor sensor6, sensor1, sensor2, sensor3, sensor4, sensor5;
public enum COLOR_DETECTED{
    RED,
    GREEN,
    BLUE,
    UNKNOWN;
}

    @Override
    public void runOpMode() {

        sensor1 = hardwareMap.get(NormalizedColorSensor.class,"clr-1");
        sensor2 = hardwareMap.get(NormalizedColorSensor.class,"clr-2");
        sensor3 = hardwareMap.get(NormalizedColorSensor.class,"clr-3");
        sensor4 = hardwareMap.get(NormalizedColorSensor.class,"clr-4");
        sensor5 = hardwareMap.get(NormalizedColorSensor.class,"clr-5");
        sensor6 = hardwareMap.get(NormalizedColorSensor.class,"clr-6");




        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA colors1 = sensor1.getNormalizedColors();
            NormalizedRGBA colors2 = sensor2.getNormalizedColors();
            NormalizedRGBA colors3 = sensor3.getNormalizedColors();
            NormalizedRGBA colors4 = sensor4.getNormalizedColors();
            NormalizedRGBA colors5 = sensor5.getNormalizedColors();
            NormalizedRGBA colors6 = sensor6.getNormalizedColors();

            telemetry.addData("r:",colors1.red);
            telemetry.addData("g:",colors1.green);
            telemetry.addData("b:",colors1.blue);
            telemetry.update();

        }
    }
}