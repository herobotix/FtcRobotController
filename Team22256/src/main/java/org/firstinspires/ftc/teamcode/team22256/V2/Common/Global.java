package org.firstinspires.ftc.teamcode.team22256.V2.Common;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.team22256.V2.Common.Subsystems.NormColorSensor;

public class Global {

    public static Pose pose = new Pose();
    public enum Alliance{
        RED,
        BLUE
    }
    public static Alliance alliance = Alliance.BLUE;
    public static NormColorSensor.COLOR[] motif = new NormColorSensor.COLOR[3];
    public static int motifIndex;
    public static double turretPosition = 0;
}
