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

public class Test extends NextFTCOpMode {

    public final MotorEx frontLeft = new MotorEx("frontLeft")
            .reversed();// reverse
    public final MotorEx backLeft = new MotorEx("backLeft");
    public final MotorEx frontRight = new MotorEx("frontRight");
    public final MotorEx backRight = new MotorEx("backRight")
            .reversed();//reverse
    private DriverControlledCommand driverControlled;

    Command waitTillAtRPM = new WaitUntil(Shooter.INSTANCE::upToSpeed);

    public Test(){
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Sorter.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE, Limelight.INSTANCE)
        );
    }
    @Override
    public void onInit(){
        Turret.INSTANCE.setState(Turret.State.IDLE);
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled = new MecanumDriverControlled(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                Gamepads.gamepad2().leftStickY().negate(),
                Gamepads.gamepad2().leftStickX(),
                Gamepads.gamepad2().rightStickX()
        );
        driverControlled.schedule();

        Turret.INSTANCE.setState(Turret.State.VISION);

        Gamepads.gamepad1().a()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Intake.INSTANCE.intaking)
                .whenBecomesFalse(Intake.INSTANCE.stop);

        Gamepads.gamepad1().b()
                .whenBecomesTrue(new SequentialGroup(
                        Sorter.INSTANCE.RK_UpDown(),
                        waitTillAtRPM,
                        Sorter.INSTANCE.LK_UpDown(),
                        waitTillAtRPM,
                        Sorter.INSTANCE.BK_UpDown()
                        )


                );

        Gamepads.gamepad1().dpadRight().whenBecomesTrue(Shooter.INSTANCE.farTriangle);
        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(Shooter.INSTANCE.closeTriangle);
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(Intake.INSTANCE.outtaking);

    }
    @Override
    public void onUpdate(){
        telemetry.addData("vel",Shooter.INSTANCE.getLeftVelocity());
        telemetry.addData("vel2", Shooter.INSTANCE.getRightVelocity());
        telemetry.addData("pos",Turret.INSTANCE.getRawTicks());
        telemetry.addData("goal",Turret.INSTANCE.getSetPoint());
        telemetry.addData("tx",Limelight.INSTANCE.getTx());
        telemetry.addData("Target found", Limelight.INSTANCE.targetFound());
        telemetry.addData("Heart beat", Limelight.INSTANCE.getHeartBeat());
        telemetry.addData("detection counter", Limelight.INSTANCE.getDetectionCount());
        telemetry.addData("error", Shooter.INSTANCE.getTarget() - Shooter.INSTANCE.getRightVelocity());
        telemetry.addData("target", Shooter.INSTANCE.getTarget());


        //telemetry.addData("goal", Shooter.target);

        telemetry.update();
    }
}
