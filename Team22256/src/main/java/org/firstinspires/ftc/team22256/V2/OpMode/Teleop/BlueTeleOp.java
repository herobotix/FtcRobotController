package org.firstinspires.ftc.team22256.V2.OpMode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Intake;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Limelight;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Shooter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Sorter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Turret;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.Component;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp
public class BlueTeleOp extends NextFTCOpMode {
    public BlueTeleOp() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Sorter.INSTANCE, Turret.INSTANCE, Limelight.INSTANCE)


        );
    }
    public final MotorEx frontLeft = new MotorEx("frontLeft");
    public final MotorEx backLeft = new MotorEx("backLeft");
    public final MotorEx frontRight = new MotorEx("frontRight");
    public final MotorEx backRight = new MotorEx("backRight");
    private DriverControlledCommand driverControlled;
    //EVO FIRECRACKER
    @Override
    public void onInit(){

    }
    @Override
    public void onWaitForStart(){

    }
    @Override
    public void onStartButtonPressed(){
        driverControlled = new MecanumDriverControlled(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );

        Gamepads.gamepad2().a()
                .whenBecomesTrue(Intake.changeIntakeMode(Intake.Mode.INTAKING))
                .whenBecomesFalse(Intake.changeIntakeMode(Intake.Mode.PAUSED));
        Gamepads.gamepad2().b()
                        .whenBecomesTrue(Intake.changeIntakeMode(Intake.Mode.OUTAKING));




        Gamepads.gamepad2().dpadLeft()
                .whenBecomesTrue(Sorter.LK_Up)
                .whenBecomesFalse(Sorter.LK_Down);
        Gamepads.gamepad2().dpadRight()
                .whenBecomesTrue(Sorter.RK_Up)
                .whenBecomesFalse(Sorter.RK_Down);
        Gamepads.gamepad2().dpadDown()
                .whenBecomesTrue(Sorter.BK_Up)
                .whenBecomesFalse(Sorter.BK_Down);

    }

    @Override
    public void onUpdate(){
    }
    @Override
    public void onStop(){

    }





}
