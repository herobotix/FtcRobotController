package org.firstinspires.ftc.team22256.V2.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Intake;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Limelight;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Shooter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Sorter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Turret;

import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToState;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;

@Autonomous

public class BlueAuto extends NextFTCOpMode {

    public final MotorEx frontLeft = new MotorEx("frontLeft")
            .reversed();// reverse
    public final MotorEx backLeft = new MotorEx("backLeft");
    public final MotorEx frontRight = new MotorEx("frontRight");
    public final MotorEx backRight = new MotorEx("backRight")
            .reversed();//reverse

    MotorGroup driveTrain = new MotorGroup(frontRight,frontLeft,backRight,backLeft);

    Command waitTillAtRPM = new WaitUntil(Shooter.INSTANCE::upToSpeed);
    Command waitTillAimed = new WaitUntil(Turret.INSTANCE::isAiming);
    Command fullShoot = new SequentialGroup(
            Sorter.INSTANCE.RK_UpDown(),
            waitTillAtRPM,
            Sorter.INSTANCE.LK_UpDown(),
            waitTillAtRPM,
            Sorter.INSTANCE.BK_UpDown()
    );

    public BlueAuto(){
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Sorter.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE, Limelight.INSTANCE)
        );
    }

    private Command autoRoutine(){
        return new SequentialGroup(
                waitTillAimed,
                Shooter.INSTANCE.farTriangle,
                waitTillAtRPM,
                fullShoot
        );
    }

    @Override
    public void onInit() {
        Turret.INSTANCE.setState(Turret.State.IDLE);
    }

    @Override
    public void onStartButtonPressed() {
        autoRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        telemetry.update();
    }
}
