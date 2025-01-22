package org.firstinspires.ftc.team22256.methods;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class arm{
    double target;
    public arm(DcMotor slide){
        slide = hardwareMap.get(DcMotor.class,"slide");

    }
    public void topPos(){
        target = -4000;
    }
    public void bottomPos(){
        target = -100;
    }
    public void raiseQuickly(){
        target = target - 2;
    }
    public void raiseSlowly(){
        target = target - 1;
    }
    public void lowerQuickly(){
        target = target + 2;
    }
    public void lowerSlowly(){
        target = target + 1;
    }


}


