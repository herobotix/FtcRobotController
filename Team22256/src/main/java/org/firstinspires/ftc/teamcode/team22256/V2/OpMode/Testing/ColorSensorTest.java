
package org.firstinspires.ftc.teamcode.team22256.V2.OpMode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.team22256.V2.Common.Commands.ShootMotif;
import org.firstinspires.ftc.teamcode.team22256.V2.Common.Global;
import org.firstinspires.ftc.teamcode.team22256.V2.Common.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.team22256.V2.Common.Subsystems.NormColorSensor;
import org.firstinspires.ftc.teamcode.team22256.V2.Common.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.team22256.V2.Common.Subsystems.Sorter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class ColorSensorTest extends NextFTCOpMode {

    {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Sorter.INSTANCE, Shooter.INSTANCE, Intake.INSTANCE)
        );
    }


    @Override
    public void onStartButtonPressed(){
        Global.motif[0] = NormColorSensor.COLOR.PURPLE;
        Global.motif[1] = NormColorSensor.COLOR.PURPLE;
        Global.motif[2] = NormColorSensor.COLOR.GREEN;

        // Gamepads.gamepad1().b().whenBecomesTrue(Shooter.INSTANCE.farTriangle);
        //Gamepads.gamepad1().x().whenBecomesTrue(Shooter.INSTANCE.closeTriangle);
        Gamepads.gamepad1().a().whenBecomesTrue(new ShootMotif());

    }

    @Override
    public void onUpdate(){
        telemetry.addData("spot 0 color", Sorter.INSTANCE.updateSpotColor(0));
        telemetry.addData("spot 1 color", Sorter.INSTANCE.updateSpotColor(1));
        telemetry.addData("spot 2 color", Sorter.INSTANCE.updateSpotColor(2));
        telemetry.update();
    }
}