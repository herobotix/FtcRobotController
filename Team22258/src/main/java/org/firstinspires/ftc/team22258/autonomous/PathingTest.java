package org.firstinspires.ftc.team22258.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team22258.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "PathingTest (Auto) [1.0.1]", group="autonomous")
public final class PathingTest extends LinearOpMode {
    public static double
        startX= 16, startY= -66,
        ChamberX= 0, ChamberY= -45,
        upX= 38, nearY= -59, farY= -16,
        mark1X= 54, mark2X= 65, mark3X= 77;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(startX, startY, -Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        // Setup & Run Actions
            Actions.runBlocking(drive.actionBuilder(beginPose)
                // Move to Chamber
                    .strafeTo(new Vector2d(ChamberX, ChamberY))

                // Move to Start Pos.
                    .strafeTo(new Vector2d(startX, startY))

                // Spline to the (Up, Far) Position
                    .setTangent(0)
                    .splineToConstantHeading(new Vector2d(upX, farY), Math.PI / 2)

                // Move To & Push "Sample" 1 into Observation Zone
                    .strafeTo(new Vector2d(mark1X, farY))
                    .strafeTo(new Vector2d(mark1X, nearY))
                    .strafeTo(new Vector2d(mark1X, farY))

                // Move To & Push "Sample" 2 into Observation Zone
                    .strafeTo(new Vector2d(mark2X, farY))
                    .strafeTo(new Vector2d(mark2X, nearY))
                    .strafeTo(new Vector2d(mark2X, farY))

                // Move To & Push "Sample" 3 into Observation Zone
                    .strafeTo(new Vector2d(mark3X, farY))
                    .strafeTo(new Vector2d(mark3X, nearY))
                    .strafeTo(new Vector2d(mark3X, farY))

                .build()); // Begin Loop
    }
}
