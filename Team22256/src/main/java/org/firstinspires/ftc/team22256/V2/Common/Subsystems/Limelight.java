package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Limelight implements Subsystem {

    public  final static Limelight INSTANCE = new Limelight();

    private static Limelight3A limelight;
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
    public static double getTx(){
        LLResult llResult = limelight.getLatestResult();
        return (llResult != null && llResult.isValid() ? llResult.getTx() : Double.NaN);
    }

    @Override
    public void initialize(){
        if (limelight != null) limelight.start();
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

}
