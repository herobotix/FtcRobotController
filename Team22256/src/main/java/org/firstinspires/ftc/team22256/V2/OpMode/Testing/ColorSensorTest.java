
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

        sensor1.setGain(4);
        sensor4.setGain(4);



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

            telemetry.addData("r1:",colors1.red);
            telemetry.addData("g1:",colors1.green);
            telemetry.addData("b1:",colors1.blue);
            telemetry.addData("r4",colors4.red);
            telemetry.addData("g4",colors4.green);
            telemetry.addData("b4",colors4.blue);
            telemetry.addData("rA",(colors4.red + colors1.red)/2);
            telemetry.addData("gA",(colors4.green + colors1.green)/2);
            telemetry.addData("bA",(colors4.blue + colors1.blue)/2);

            telemetry.update();

        }
    }
}