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

    private static MotorEx sl = new MotorEx("shooterLeft").reversed().floatMode();
    private static MotorEx sr = new MotorEx("shooterRight").floatMode();
    public static MotorGroup shooterGroup = new MotorGroup(sr, sl);

    public static double target = 0;
    public static double tolerance = 40;

    public static double distance;
    public static double kP = 0.003;
    public static double kS = 0.04;
    public static double kV = 0.000316;
    public static double flywheelTarget = 0;
    public static  double currentVelocity = 0;




    public static double getShooterVelocity() {
        return shooterGroup.getVelocity();

    }

    public static boolean upToSpeed() {
        return getShooterVelocity() >= target - tolerance;
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
