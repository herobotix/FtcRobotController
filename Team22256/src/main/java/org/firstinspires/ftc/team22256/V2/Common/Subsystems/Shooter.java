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
    public static final Shooter INSTANCE = new Shooter();
    private Shooter(){
         controlSystem = ControlSystem.builder()
                .velPid(0.000001,0,0)
                 .basicFF(0.002251
                 )
                .build();
         controlSystem.setGoal(new KineticState(0,0));
    }

    private ControlSystem controlSystem;
    private final MotorEx shooterLeft = new MotorEx("shooterLeft");
    private final MotorEx shooterRight = new MotorEx("shooterRight");

    private double target = 0;
    private double tolerance = 40;

    private double distance;

    public double getLeftVelocity() {
        return shooterLeft.getVelocity();
    }
    public double getRightVelocity() {
        return shooterRight.getVelocity();
    }

    public boolean upToSpeed() {
        return getRightVelocity() >= target - tolerance;
    }
    public Command farTriangle = new LambdaCommand()
            .setStart(() -> {
                controlSystem.setGoal(new KineticState(0,
                        270));
            });
    //short:200

    public Command closeTriangle = new LambdaCommand()
            .setStart(() -> {
                controlSystem.setGoal(new KineticState(0,215));
            })
            .setIsDone(() -> true);

    public double getTarget() {
        return target;
    }

    @Override
    public void initialize(){
        controlSystem.setGoal(new KineticState(0,0));
    }

    @Override
    public void periodic() {
        shooterLeft.setPower(-controlSystem.calculate(shooterRight.getState()));
        shooterRight.setPower(controlSystem.calculate(shooterRight.getState()));

        target = controlSystem.getGoal().getVelocity() * 5.939707;
    }
}
