package org.firstinspires.ftc.team22256.V2.OpMode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Intake;

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
                new SubsystemComponent(Intake.INSTANCE)
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
                .whenTrue(new SetPower(frontRight,1))
                .whenFalse(new SetPower(frontRight,0)
                );

        Gamepads.gamepad1().x()
                .whenTrue(new SetPower(backRight,1))
                .whenFalse(new SetPower(backRight,0)
                );

        Gamepads.gamepad1().y()
                .whenTrue(new SetPower(backLeft,1))
                .whenFalse(new SetPower(backLeft,0)
                );

    }
    @Override
    public void onUpdate(){

    }
}
