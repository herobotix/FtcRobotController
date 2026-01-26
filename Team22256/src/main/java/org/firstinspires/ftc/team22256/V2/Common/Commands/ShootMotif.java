package org.firstinspires.ftc.team22256.V2.Common.Commands;

import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Intake;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Shooter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Sorter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;

public class ShootMotif extends Command {

    Command waitTillAtPRM;
    Command isAimed;
    public ShootMotif(){
        requires(Sorter.INSTANCE);
        waitTillAtPRM = new WaitUntil(Shooter::upToSpeed);

    }

    @Override
    public boolean isDone(){
        return false;
    }

    @Override
    public void start(){
        Sorter.updateAllSpots();
        Sorter.buildKickQueue();
        for(int i = 0;i < 3;i++){
            if(Sorter.kickQueue.get(i) == 0){
                new SequentialGroup(
                        waitTillAtPRM,
                        Sorter.LK_UpDown(),
                        new Delay(0.1)
                ).schedule();
            if(Sorter.kickQueue.get(i) == 1){
                new SequentialGroup(
                        waitTillAtPRM,
                        Sorter.RK_UpDown(),
                        new Delay(0.1)
                ).schedule();
                }
            if(Sorter.kickQueue.get(i) == 2){
                new SequentialGroup(
                        waitTillAtPRM,
                        Sorter.RK_UpDown(),
                        new Delay(0.1)
                ).schedule();
                }
            }
        }

    }

    @Override
    public void update(){

    }

    @Override
    public void stop(boolean interrupted){
        Intake.changeIntakeMode(Intake.Mode.INTAKING);
    }
}
