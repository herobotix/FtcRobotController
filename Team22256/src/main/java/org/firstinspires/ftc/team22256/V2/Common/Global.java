package org.firstinspires.ftc.team22256.V2.Common;

import com.pedropathing.geometry.Pose;

public class Global {

    public static Pose pose = new Pose();
    public enum Alliance{
        RED,
        BLUE
    }
    public static Alliance alliance = Alliance.RED;
    public static int MotifID = 0;
}
