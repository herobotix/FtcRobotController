package org.firstinspires.ftc.team22256.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team22256.RR.MecanumDrive;
import org.firstinspires.ftc.team22256.Experimental.roadrunner_experimental;
@Config
@Autonomous
public final class PathingTest extends LinearOpMode {
    public static double
        startX= 0, startY= 0,
        deltaX = 48, deltaY = 0,
        fudF = 1, fudB = 1.05,
        fudR = 1, fudL = 1,
        fudFI = 1, fudBI= -2.7,
        fudRI = 1, fudLI = 1,


        //x = actual, y = theoretical,apply quadratic best fit
        fudBA = -0.00067, fudBB = 0.97, fudBC = -0.28,
        fudFA = 0.00076, fudFB = 0.95, fudFC =-0.13,
        fudRA = 1, fudRB = 1, fudRC =1,
        fudLA =-0.002132, fudLB = 1.815 , fudLC = 0.8388
    ;

    
    @Override
    public void runOpMode() throws InterruptedException {
      roadrunner_experimental drive = new roadrunner_experimental();
      // Setup Robot Position
       // Pose2d beginPose = new Pose2d(startX, startY, 0);
      //  MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        
        waitForStart();

        drive.strafe2(2,5);
      // Create & Run Actions
   /*     Actions.runBlocking(drive.actionBuilder(beginPose)
          // Move to Chamber
            .strafeTo(new Vector2d(
                    deltaX >= 0 ?
                            fudFA * Math.pow(deltaX,2) + fudFB*deltaX + fudFC:
                            fudBA * Math.pow(deltaX,2)+ fudBB * deltaX + fudBC,
                    deltaY >= 0 ?
                            fudLA * Math.pow(deltaY,2) + fudLB * deltaY + fudLC :
                            fudR * deltaY + fudRI))
          // Begin Loop
            .build())
        ;

    */
    }
}
