package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();
    private Shooter(){ }

    public static final MotorEx shooterMotor = new MotorEx("shooter");
    public static double target = 1700;
    public static final double farPower = 0;
    public static final double shortPower = 0;

    public static double tolerance = 40;
    public static double kP;
    public static double kV;
    public static double kS;

    public static double getVelocity() {
        return shooterMotor.getVelocity();
    }

    public static boolean upToSpeed() {
        return getVelocity() >= target - tolerance;
    }
    public static Command setFarPower(double power){
        return new InstantCommand(() -> shooterMotor.setPower(farPower));
    }
    public static Command setShortPower(double power){
        return new InstantCommand(() -> shooterMotor.setPower(shortPower));
    }

    public static Command shoot3Far() {
        return new SequentialGroup(
                shootFarBK(),
                shootFarLK(),
                shootFarRK()
        );
    }
    public static Command shoot3Short() {
        return new SequentialGroup(
                shootShortBk(),
                shootShortLk(),
                shootShortRk()
        );
    }


    private static Command shootFarBK() {
        return new SequentialGroup(
                new WaitUntil(Shooter::upToSpeed),
                Sorter.BK_Up,
                Sorter.BK_Down
        );
    }
    private static Command shootShortBk() {
        return new SequentialGroup(
                new WaitUntil(Shooter::upToSpeed),
                Sorter.BK_Up,
                Sorter.BK_Down
        );
    }
    private static Command shootFarLK() {
        return new SequentialGroup(
                new WaitUntil(Shooter::upToSpeed),
                Sorter.LK_Up,
                Sorter.LK_Down
        );
    }
    private static Command shootShortLk() {
        return new SequentialGroup(
                new WaitUntil(Shooter::upToSpeed),
                Sorter.LK_Up,
                Sorter.LK_Down
        );
    }
    private static Command shootFarRK() {
        return new SequentialGroup(
                new WaitUntil(Shooter::upToSpeed),
                Sorter.BK_Up,
                Sorter.BK_Down
        );
    }
    private static Command shootShortRk() {
        return new SequentialGroup(
                new WaitUntil(Shooter::upToSpeed),
                Sorter.BK_Up,
                Sorter.BK_Down
        );
    }


    @Override
    public void periodic() {
    }

}
