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
        startH = 0,
        
        // Deltas
        deltaX = 0,
        deltaY = 0,
        deltaH = 0,
        
        //fudge factors
        ffFa = 0.9046, ffFb = 1.0060,
        ffBa = 0.8952, ffBb = 1.0090,
        ffLa = 2.0240, ffLb = 0.8703,
        ffRa = 1.6260, ffRb = 0.9174
    ;
    
    @Override
    public void runOpMode() throws InterruptedException {
      
        // Setup Robot Position
        Pose2d beginPose = new Pose2d(startX, startY, startH);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        
        waitForStart();
        
        // Create & Run Actions
        Actions.runBlocking(drive.actionBuilder(beginPose)
            // Move to Delta Position
            .strafeToLinearHeading(
                new Vector2d(
                    (deltaX>=0)?(ffFa*Math.pow(deltaY,ffFb)):(-ffBa*Math.pow(-deltaY,ffBb)),
                    (deltaY>=0)?(ffLa*Math.pow(deltaY,ffLb)):(-ffRa*Math.pow(-deltaY,ffRb))
                ),  (deltaH)
            )
            
            // Begin Loop
            .build())
        ;
    }
}
