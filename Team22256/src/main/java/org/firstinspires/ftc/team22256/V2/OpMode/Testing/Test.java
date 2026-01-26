package org.firstinspires.ftc.team22256.V2.OpMode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team22256.V2.Common.Commands.ShootMotif;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Intake;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Shooter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Sorter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Turret;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.delays.Delay;
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

    public Test(){
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Sorter.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE)
        );
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

        ShootMotif shootMotif = new ShootMotif();

        Gamepads.gamepad1().a()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Intake.changeIntakeMode(Intake.Mode.INTAKING))
                .whenBecomesFalse(Intake.changeIntakeMode(Intake.Mode.PAUSED)
        );
/*
        Gamepads.gamepad1().b()

                .whenBecomesTrue(Sorter.RK_Up)
                .whenBecomesFalse(Sorter.RK_Down
                );
*/
        Gamepads.gamepad1().b()

                .whenBecomesTrue(new SequentialGroup(
                        Sorter.RK_UpDown(),
                        new Delay(0.2),
                        Sorter.LK_UpDown(),
                        new Delay(0.2),
                        Sorter.BK_UpDown()
                        )


                );
        Gamepads.gamepad1().x()

                .whenBecomesTrue(Sorter.LK_Up)
                .whenBecomesFalse(Sorter.LK_Down
                );

        Gamepads.gamepad1().y()
                .whenBecomesTrue(Sorter.BK_Up)
                .whenBecomesFalse(Sorter.BK_Down
                );
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(Shooter.farTriangle);
        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(Shooter.closeTriangle);
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(Intake.changeIntakeMode(Intake.Mode.OUTAKING));

    }
    @Override
    public void onUpdate(){
        telemetry.addData("vel",Shooter.getLeftVelocity());
        telemetry.addData("vel2", Shooter.getRightVelocity());
        telemetry.update();
    }
}
