package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class NormColorSensor {
    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors;
    public enum COLOR{
        PURPLE, //1
        GREEN, //2
        EMPTY; //0
    }

    public NormColorSensor(HardwareMap hardwareMap,String name){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class,name);
        colorSensor.setGain(4);
    }
    public float getRed(){
        return colors.red;
    }
    public float getGreen(){
        return colors.green;
    }
    public float getBlue(){
        return colors.blue;
    }
    public void updateColors(){
        colors = colorSensor.getNormalizedColors();
    }

    public COLOR getDetectedColor(){
        NormalizedRGBA colors= colorSensor.getNormalizedColors();
        float normRed = colors.red;
        float normGreen = colors.green;
        float normBlue = colors.blue;
        if(normBlue > normGreen && normRed > (0.5 * normGreen) && normRed > 0.01){
            return COLOR.PURPLE; //purple
        } else if(normGreen > 0.03 && normGreen > (normRed + 0.01) && normGreen > (normBlue + 0.003)){
            return COLOR.GREEN; //green
        } else{
            return COLOR.EMPTY;//none
        }
    }
}














