package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

import java.util.List;

public class Limelight implements Subsystem {

    public  final static Limelight INSTANCE = new Limelight();
    private static Limelight3A limelight;
    private Limelight(){}
    private static boolean targetFound = false;
    static double tx = 0.0;
    public static int APRIL_TAG_BLUE_ID = 20;
    public static int APRIL_TAG_RED_ID = 24;

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
        return tx;
    }

    public static boolean targetFound(){
        return targetFound;
    }

    @Override
    public void initialize(){
        if (limelight != null) limelight.start();
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

    }

    @Override
    public void periodic(){
    /*
        targetFound = false;
        tx =0.0;

        LLResult llResult = limelight.getLatestResult();
        if(llResult != null && llResult.isValid()){
            List<LLResultTypes.FiducialResult> fidcuial = llResult.getFiducialResults();
            if(!fidcuial.isEmpty()){

                for(LLResultTypes.FiducialResult fidcial : fidcuial){
                    int aprilTagId = fidcial.getFiducialId();

                    if(aprilTagId == APRIL_TAG_BLUE_ID){
                        targetFound = true;
                        tx = fidcial.getFiducialId();
                    }
                }
            }
        }
        */
    }

}
