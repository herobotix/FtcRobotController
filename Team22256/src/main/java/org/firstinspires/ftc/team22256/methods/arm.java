package org.firstinspires.ftc.team22256.methods;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

public class arm{
    private DcMotor slide;
    public static double target = 0;
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


