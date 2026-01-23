package org.firstinspires.ftc.team22256.V2.OpMode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Intake;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Sorter;

import dev.nextftc.bindings.BindingManager;
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
                new SubsystemComponent(Intake.INSTANCE, Sorter.INSTANCE)
        );
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled = new MecanumDriverControlled(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();

        Gamepads.gamepad1().a()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Intake.changeIntakeMode(Intake.Mode.INTAKING))
                .whenBecomesFalse(Intake.changeIntakeMode(Intake.Mode.PAUSED)
        );

        Gamepads.gamepad1().b()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Sorter.RK_Up)
                .whenBecomesFalse(Sorter.RK_Down
                );

        Gamepads.gamepad1().x()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Sorter.LK_Up)
                .whenBecomesFalse(Sorter.LK_Down
                );

        Gamepads.gamepad1().y()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(Sorter.BK_Up)
                .whenBecomesFalse(Sorter.BK_Down
                );

    }
    @Override
    public void onUpdate(){

    }
}
