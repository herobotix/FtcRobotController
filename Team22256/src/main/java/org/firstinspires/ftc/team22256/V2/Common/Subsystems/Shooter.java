package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

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
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;


public class Shooter implements Subsystem {
    static ControlSystem controlSystem;
    public static final Shooter INSTANCE = new Shooter();
    private Shooter(){
         controlSystem = ControlSystem.builder()
                .velPid(0.0005,0,0)
                 .basicFF(0.005)
                .build();
         controlSystem.setGoal(new KineticState(0,0));
    }

    public static final MotorEx shooterLeft = new MotorEx("shooterLeft");
    public static final MotorEx shooterRight = new MotorEx("shooterRight");

    public static double target = 0;

    public static double distance;

    public static double tolerance = 40;
    public static double getLeftVelocity() {
        return shooterLeft.getVelocity();
    }
    public static double getRightVelocity() {
        return shooterRight.getVelocity();
    }

    public static boolean upToSpeed() {
        return (Math.abs(getLeftVelocity())+Math.abs(getRightVelocity()))/2 >= target - tolerance;
    }
    public static Command farTriangle = new LambdaCommand()
            .setStart(() -> {
                controlSystem.setGoal(new KineticState(0,
                        265));
            });
    //short:200

    public static Command closeTriangle = new LambdaCommand()
            .setStart(() -> {
                controlSystem.setGoal(new KineticState(0,215));
            })
            .setIsDone(() -> true);



    @Override
    public void periodic() {
        shooterLeft.setPower(-controlSystem.calculate(shooterRight.getState()));
        shooterRight.setPower(controlSystem.calculate(shooterRight.getState()));
        target = shooterRight.getVelocity();
    }

}
