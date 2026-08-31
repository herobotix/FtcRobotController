package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;


public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake(){ }
    private MotorEx intake = new MotorEx("intake");

    private double intakePower = -1;
    private double outtakePower = 1;
    private final int stopPower = 0;
    private boolean changesMade = false;
    public enum Mode{
        INTAKING,
        OUTAKING,
        PAUSED;
    }

    public void setIntakePower(double intakePower1) {
        intakePower = intakePower1;
    }
    public void setOuttakePower(double outtakePower1){
        outtakePower = outtakePower1;
    }

    public Command intaking = new LambdaCommand()
            .setStart(() -> intake.setPower(intakePower))
            .setIsDone(() -> true);
    public Command outtaking = new LambdaCommand()
            .setStart(() -> intake.setPower(outtakePower))
            .setIsDone(() -> true);

    public Command stop = new LambdaCommand()
            .setStart(() -> intake.setPower(stopPower))
            .setIsDone(() -> true);
    public double getIntakePower() {
        return intakePower;
    }

    public double getOuttakePower() {
        return outtakePower;
    }

    @Override
    public void initialize(){
        stop.schedule();
    }
    //make sure to make intaking on start in op mode

}






