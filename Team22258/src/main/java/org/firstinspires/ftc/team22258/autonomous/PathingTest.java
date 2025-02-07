package org.firstinspires.ftc.team22258.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team22258.roadrunner.MecanumDrive;
import org.firstinspires.ftc.team22258.roadrunner.TankDrive;
import org.firstinspires.ftc.team22258.roadrunner.tuning.TuningOpModes;

@Config
public final class PathingTest extends LinearOpMode {
    public static double finalX=36, finalY=-26;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(16, -66, -Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(beginPose)
                .setTangent(0)
                .splineTo(new Vector2d(finalX, finalY), Math.PI / 2)
                //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                //.splineToLinearHeading(new Pose2d(0, 60, 0), Math.PI / 2)
                .build());
    }
}
