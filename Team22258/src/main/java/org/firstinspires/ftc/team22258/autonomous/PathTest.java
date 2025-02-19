package org.firstinspires.ftc.team22258.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team22258.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "PathTest [1.0.0]", group="autonomous")
public final class PathTest extends LinearOpMode {
    public static double
      // Start Position
        startX = 0,
        startY = 0,
        
      // Deltas
        deltaX = 0,
        deltaY = 0,
        
      //fudge factors
        ffFa = 0.00009272, ffFb = 0.9221, ffFc = 0,
        ffBa = -0.0001061, ffBb = 0.9187, ffBc = 0,
        ffLa = 0, ffLb = 1, ffLc = 0,
        ffRa = 0, ffRb = 1, ffRc = 0
    ;
    
    @Override
    public void runOpMode() throws InterruptedException {
      
      // Setup Robot Position
        Pose2d beginPose = new Pose2d(startX, startY, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        
        waitForStart();
        
      // Create & Run Actions
        Actions.runBlocking(drive.actionBuilder(beginPose)
          // Move to Chamber
            .strafeTo(new Vector2d(
                (deltaX>=0)?((ffFa*deltaX*deltaX)+(ffFb*deltaX)+(ffFc)):((ffBa*deltaX*deltaX)+(ffBb*deltaX)+(ffBc)),
                (deltaY>=0)?((ffLa*deltaY*deltaY)+(ffLb*deltaY)+(ffLc)):((ffRa*deltaY*deltaY)+(ffRb*deltaY)+(ffRc))
            ))
            
          // Begin Loop
            .build())
        ;
    }
}
