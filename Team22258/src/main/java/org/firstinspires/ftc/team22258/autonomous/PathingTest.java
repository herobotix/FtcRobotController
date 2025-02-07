package org.firstinspires.ftc.team22258.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team22258.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "PathingTest", group="autonomous")
public final class PathingTest extends LinearOpMode {
    public static double startX=16,startY=-66,nearY=-52, farY=-16, mark1X=54, mark2X=65, mark3X=78, upX=38;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(startX, startY, -Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(beginPose)
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(upX, farY), Math.PI / 2)
                .strafeTo(new Vector2d(mark1X, farY))
                    .strafeTo(new Vector2d(mark1X, nearY))
                    .strafeTo(new Vector2d(mark1X, farY))
                .strafeTo(new Vector2d(mark2X, farY))
                    .strafeTo(new Vector2d(mark2X, nearY))
                    .strafeTo(new Vector2d(mark2X, farY))
                .strafeTo(new Vector2d(mark3X, farY))
                    .strafeTo(new Vector2d(mark3X, nearY))
                    .strafeTo(new Vector2d(mark3X, farY))
                .strafeTo(new Vector2d(upX, farY))
                //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                //.splineToLinearHeading(new Pose2d(0, 60, 0), Math.PI / 2)
                .build());
    }
}
