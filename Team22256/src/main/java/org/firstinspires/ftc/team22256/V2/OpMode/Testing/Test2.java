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


    public Test2(){
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(Turret.INSTANCE,Limelight.INSTANCE)
        );
    }



    @Override
    public void onStartButtonPressed() {
    }

    @Override
    public void onUpdate(){
        telemetry.addData("pos",Turret.INSTANCE.getRawTicks());
        telemetry.addData("goal",Turret.INSTANCE.getSetPoint());
        telemetry.addData("tx",Limelight.INSTANCE.getTx());
        telemetry.addData("Target found", Limelight.INSTANCE.targetFound());
        telemetry.addData("Heart beat", Limelight.INSTANCE.getHeartBeat());
        telemetry.addData("detection counter", Limelight.INSTANCE.getDetectionCount());
        telemetry.update();
    }

    @Override
    public void onStop(){
        Limelight.INSTANCE.stopLimelight();
    }
}
