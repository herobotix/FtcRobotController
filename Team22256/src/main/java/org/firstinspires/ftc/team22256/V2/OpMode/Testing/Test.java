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

    Command waitTillAtRPM = new WaitUntil(Shooter::upToSpeed);

    public Test(){
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Sorter.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE, Limelight.INSTANCE)
        );
    }
    @Override
    public void onInit(){
        Turret.state= Turret.State.IDLE;
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

        Turret.state = Turret.State.VISION;

        Gamepads.gamepad1().a()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Intake.intaking)
                .whenBecomesFalse(Intake.stop
        );

        Gamepads.gamepad1().b()
                .whenBecomesTrue(new SequentialGroup(
                        Sorter.RK_UpDown(),
                        waitTillAtRPM,
                        Sorter.LK_UpDown(),
                        waitTillAtRPM,
                        Sorter.BK_UpDown()
                        )


                );

        Gamepads.gamepad1().dpadRight().whenBecomesTrue(Shooter.farTriangle);
        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(Shooter.closeTriangle);
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(Intake.outtaking);

    }
    @Override
    public void onUpdate(){
        telemetry.addData("vel",Shooter.getLeftVelocity());
        telemetry.addData("vel2", Shooter.getRightVelocity());
        telemetry.addData("pos",Turret.turret.getRawTicks());
        telemetry.addData("goal",Turret.controller0.getSetPoint());
        telemetry.addData("tx",Limelight.getTx());
        telemetry.addData("Target found", Limelight.targetFound());
        telemetry.addData("Heart beat", Limelight.getHeartBeat());
        telemetry.addData("detection counter", Limelight.getDetectionCount());
        telemetry.addData("error", Shooter.target - Shooter.getRightVelocity());
        telemetry.addData("target", Shooter.target);


        //telemetry.addData("goal", Shooter.target);

        telemetry.update();
    }
}
