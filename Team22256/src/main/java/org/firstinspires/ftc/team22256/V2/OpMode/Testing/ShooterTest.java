package org.firstinspires.ftc.team22256.V2.OpMode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team22256.V2.Common.Commands.ShootMotif;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Intake;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Limelight;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Shooter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Sorter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Turret;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@TeleOp

public class ShooterTest extends NextFTCOpMode {
    Command waitTillAtRPM = new WaitUntil(Shooter::upToSpeed);
    public ShooterTest(){
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Shooter.INSTANCE,Sorter.INSTANCE)
        );
    }
    @Override
    public void onInit(){

    }

    @Override
    public void onStartButtonPressed() {


        Gamepads.gamepad1().a().whenBecomesTrue(
                new SequentialGroup(
                        waitTillAtRPM,
                        Sorter.RK_UpDown(),
                        waitTillAtRPM,
                        Sorter.BK_UpDown())

        );





    }
    @Override
    public void onUpdate(){


        telemetry.addData("velocity",Shooter.getShooterVelocity());
        telemetry.addData("reference",Shooter.flywheelTarget);
        telemetry.update();
    }
}
