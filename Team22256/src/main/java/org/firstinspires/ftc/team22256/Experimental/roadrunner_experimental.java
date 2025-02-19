package org.firstinspires.ftc.team22256.Experimental;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.team22256.RR.MecanumDrive;

public class roadrunner_experimental {

    public static double
            deltaX =0, deltaY =0,
            fudBA = -0.00067, fudBB = 0.97, fudBC = -0.28,
            fudFA = 0.00076, fudFB = 0.95, fudFC =-0.13,
            fudRA = 1, fudRB = 1, fudRC =1,
            fudLA =-0.002132, fudLB = 1.815 , fudLC = 0.8388;
    double RR_P = 0;
    double A_P = 0;
    Pose2d beginPose = new Pose2d(0, 0, 0);
    MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
    public void strafe2(double deltaX,double deltaY){
        Actions.runBlocking(drive.actionBuilder(beginPose)
        .strafeTo(new Vector2d(
                deltaX >= 0 ?
                        fudFA * Math.pow(deltaX,2) + fudFB * deltaX + fudFC:
                        fudBA * Math.pow(deltaX,2) + fudBB * deltaX + fudBC,
                deltaY >= 0 ?
                        fudLA * Math.pow(deltaY,2) + fudLB * deltaY + fudLC :
                        fudRA * Math.pow(deltaY,2) + fudRB * deltaY + fudRC))
                .build());
    }
    public void strafe2LinearHeading(){

    }
}
