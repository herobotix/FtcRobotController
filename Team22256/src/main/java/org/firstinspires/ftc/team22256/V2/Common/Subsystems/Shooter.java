package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team22256.V2.Common.Global;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;


@Configurable
public class Shooter implements Subsystem {

    public static final Shooter INSTANCE = new Shooter();
    private Shooter(){


    }

    private MotorEx sl = new MotorEx("shooterLeft").reversed().floatMode();
    private MotorEx sr = new MotorEx("shooterRight").floatMode();
    private MotorGroup shooterGroup = new MotorGroup(sr, sl);


    private final double tolerance = 21;

    public static double distance;
    public final double kP = 0.003;
    public final double kS = 0.04;
    public final double kV = 0.0003165;
    public static double flywheelTarget = 0;
    private double currentVelocity = 0;


    public double getFlywheelTarget() {
        return flywheelTarget;
    }

    public void setFlywheelTarget(double flywheelTarget) {
        this.flywheelTarget = flywheelTarget;
    }

    public double getShooterVelocity() {
        return shooterGroup.getVelocity();

    }

    public  boolean upToSpeed() {
        return getShooterVelocity() >= flywheelTarget - tolerance;
    }



    @Override
    public void initialize(){

    }

    @Override
    public void periodic() {
        currentVelocity = getShooterVelocity();

        double power = ((kP*(flywheelTarget-currentVelocity))+(kV*flywheelTarget)+(kS*Math.signum(flywheelTarget)));
        shooterGroup.setPower(power);
    }

}
