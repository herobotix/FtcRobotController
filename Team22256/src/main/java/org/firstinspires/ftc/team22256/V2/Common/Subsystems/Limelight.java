package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.team22256.V2.Common.Global;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

import java.util.List;

public class Limelight implements Subsystem {

    public final static Limelight INSTANCE = new Limelight();
    private Limelight(){}

    private Limelight3A limelight;
    private boolean targetFound = false;
    private double tx = 0.0;
    public int APRIL_TAG_BLUE_ID = 20;
    public int APRIL_TAG_RED_ID = 24;
    public int heartBeat = 0;
    public int detectionCount = 0;

    public Command intializeLimelight(){
        return new LambdaCommand()
                .setStart(() -> limelight.start())
                .setIsDone(() -> true)
                .requires(this);
    }
    public void stopLimelight(){
        limelight.stop();
    }
    public Command setPipeline(int pipeline){
        return new LambdaCommand()
                .setStart(() -> limelight.pipelineSwitch(pipeline))
                .setIsDone(() -> true)
                .requires(this);
    }
    public double getTx(){
        return tx;
    }

    public boolean targetFound(){
        return targetFound;
    }

    @Override
    public void initialize(){
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        if (limelight != null) limelight.start();
        limelight.pipelineSwitch(0);

    }
    public int getHeartBeat(){
        return heartBeat;
    }
    public int getDetectionCount(){
        return detectionCount;
    }

    @Override
    public void periodic(){

        heartBeat++;
        targetFound = false;
        tx =0.0;

        LLResult llResult = limelight.getLatestResult();
        if(llResult != null && llResult.isValid()){
            List<LLResultTypes.FiducialResult> fidcuial = llResult.getFiducialResults();
            detectionCount = fidcuial.size();
            if(!fidcuial.isEmpty()){

                for(LLResultTypes.FiducialResult fidcial : fidcuial){
                    int aprilTagId = fidcial.getFiducialId();

                    if(Global.alliance == Global.Alliance.RED){
                        if(aprilTagId == APRIL_TAG_RED_ID){
                            targetFound = true;
                            tx = fidcial.getTargetXDegrees();
                        }
                    } else if (Global.alliance == Global.Alliance.BLUE) {
                        if(aprilTagId == APRIL_TAG_BLUE_ID){
                            targetFound = true;
                            tx = fidcial.getTargetXDegrees();
                        }
                    }
                }
            }
        } else {
            detectionCount = 0;
        }
    }
}
