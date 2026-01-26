package org.firstinspires.ftc.team22256.V2.OpMode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team22256.V2.Common.Commands.ShootMotif;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Intake;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Limelight;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Shooter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Sorter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Turret;

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
public class Test2 extends NextFTCOpMode {

    public final MotorEx frontLeft = new MotorEx("frontLeft")
            .reversed();// reverse
    public final MotorEx backLeft = new MotorEx("backLeft");
    public final MotorEx frontRight = new MotorEx("frontRight");
    public final MotorEx backRight = new MotorEx("backRight")
            .reversed();//reverse
    private DriverControlledCommand driverControlled;

    public Test2(){
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Turret.INSTANCE)
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



    }
    @Override
    public void onUpdate(){
        telemetry.addData("pos",Turret.getTurretPosition());
        telemetry.addData("tx", Limelight.getTx());
        telemetry.update();
    }
}
