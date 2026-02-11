package org.firstinspires.ftc.team22256.V2.Common.Commands;

import org.firstinspires.ftc.team22256.V2.Common.Global;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Intake;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.NormColorSensor;
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

    public ShootMotif() {
        requires(Sorter.INSTANCE);
        waitTillAtPRM = new WaitUntil(Shooter.INSTANCE::upToSpeed);
    }

    @Override
    public boolean isDone(){
        return false;
    }

    @Override
    public void start(){
        //0 = left, 1 = right, 2 = back

        Sorter.INSTANCE.updateAllSpots();

        for(int i = 0; i < 3; i++){      //i is for motif index
            for(int j = 0; j < 3; j++) {  // j is for ball slot on robot
                if(Sorter.INSTANCE.storedColors[j] == Global.motif[i]){  // check if one of the slots has that color
                    if(j == 0 && !Sorter.INSTANCE.scheduled[0]){
                        new SequentialGroup(
                                waitTillAtPRM,
                                Sorter.INSTANCE.LK_UpDown()
                        ).schedule();
                        Sorter.INSTANCE.scheduled[0] = true;
                        break;
                    }
                    if(j == 1 && !Sorter.INSTANCE.scheduled[1]){
                        new SequentialGroup(
                                waitTillAtPRM,
                                Sorter.INSTANCE.RK_UpDown()
                        ).schedule();
                        Sorter.INSTANCE.scheduled[1] = true;
                        break;
                    }
                    if(j == 2 && !Sorter.INSTANCE.scheduled[2]){
                        new SequentialGroup(
                                waitTillAtPRM,
                                Sorter.INSTANCE.BK_UpDown()
                        ).schedule();
                        Sorter.INSTANCE.scheduled[2] = true;
                        break;
                    }
                }

            }
        }

        for(int j = 0; j < 3; j++) { //this loop is for catching any exceptions
            if(!Sorter.INSTANCE.scheduled[j] &&
                Sorter.INSTANCE.storedColors[j] != NormColorSensor.COLOR.EMPTY) {

                if(j == 0) {
                    new SequentialGroup(
                            waitTillAtPRM,
                            Sorter.INSTANCE.LK_UpDown()
                    ).schedule();
                    Sorter.INSTANCE.scheduled[0] = true;
                    break;
                }

                if(j == 1) {
                    new SequentialGroup(
                            waitTillAtPRM,
                            Sorter.INSTANCE.RK_UpDown()
                    ).schedule();
                    Sorter.INSTANCE.scheduled[1] = true;
                    break;
                }

                if(j == 2) {
                    new SequentialGroup(
                            waitTillAtPRM,
                            Sorter.INSTANCE.BK_UpDown()
                    ).schedule();
                    Sorter.INSTANCE.scheduled[2] = true;
                    break;
                }
            }
        }
    }

    @Override
    public void update() {

    }

    @Override
    public void stop(boolean interrupted) {
        for(int i = 0; i < 3; i++){
            Sorter.INSTANCE.scheduled[i] = false;
        }
        //Intake.INSTANCE.changeIntakeMode(Intake.Mode.INTAKING);
    }
}
