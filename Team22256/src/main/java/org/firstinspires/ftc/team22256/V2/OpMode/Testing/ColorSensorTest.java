
package org.firstinspires.ftc.team22256.V2.OpMode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Sorter;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;


@TeleOp

public class ColorSensorTest extends NextFTCOpMode {

    {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Sorter.INSTANCE)
        );
    }

    @Override
    public void onUpdate(){

        telemetry.addData("spot 0 color", Sorter.updateSpotColor(0));
        telemetry.addData("spot 1 color", Sorter.updateSpotColor(1));
        telemetry.addData("spot 2 color", Sorter.updateSpotColor(2));
        telemetry.update();

    }


}