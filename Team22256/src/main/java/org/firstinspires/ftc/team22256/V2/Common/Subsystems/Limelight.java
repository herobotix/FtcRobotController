package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;

public class Limelight implements Subsystem {

    public static final Limelight INSTANCE = new Limelight();

    private Limelight3A limelight;
    private Limelight(){}

    public Command intializeLimelight(){
        return new LambdaCommand()
                .setStart(() -> limelight.start())
                .setIsDone(() -> true)
                .requires(this);
    }
    public Command stopLimelight(){
        return new LambdaCommand()
                .setStart(() -> limelight.stop())
                .setIsDone(() -> true)
                .requires(this);
    }
    public Command setPipeline(int pipeline){
        return new LambdaCommand()
                .setStart(() -> limelight.pipelineSwitch(pipeline))
                .setIsDone(() -> true)
                .requires(this);
    }
    public double getTx(){
        LLResult llResult = limelight.getLatestResult();
        return (llResult != null && llResult.isValid() ? llResult.getTx() : Double.NaN);
    }

}
