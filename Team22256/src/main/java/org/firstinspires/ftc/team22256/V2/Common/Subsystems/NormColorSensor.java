package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class NormColorSensor {
    NormalizedColorSensor colorSensor;
    public enum COLOR_DETECTED{
        PURPLE, //1
        GREEN, //2
        NONE; //0
    }

    public NormColorSensor(HardwareMap hardwareMap,String name){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class,name);
    }
    public int getDetectedColor(){
        NormalizedRGBA colors= colorSensor.getNormalizedColors();
        float normRed = colors.red / colors.alpha;
        float normGreen = colors.green / colors.alpha;
        float normBlue = colors.blue / colors.alpha;
        if(normRed < 0 && normGreen < 0 && normBlue < 0 ){
            return 1; //purple
        } else if(normRed < 0 && normGreen < 0 && normBlue < 0 ){
            return 2; //green
        } else{
            return 0;//none
        }

    }

}














