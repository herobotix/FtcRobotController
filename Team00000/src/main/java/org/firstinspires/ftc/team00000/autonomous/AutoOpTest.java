package org.firstinspires.ftc.team00000.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.team00000.roadrunner.MecanumDrive;

@Config
@Autonomous(name="AutoOpTest", group="autonomous")
//@Disabled
public class AutoOpTest extends LinearOpMode {
    public static double Xi=0, Yi=0, θi=0, ΔX=0, ΔY=0, Δθ=0;
    public static double    ffFa=0, ffFb=1, ffFc=0,
                            ffBa=0, ffBb=1, ffBc=0,
                            ffLa=0, ffLb=1, ffLc=0,
                            ffRa=0, ffRb=1, ffRc=0;
    public static

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(Xi, Yi, Math.toRadians(θi));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(
                                ΔX>=0 ? (ffFa*ΔX*ΔX)+(ffFb*ΔX)+(ffFc) : (ffBa*ΔX*ΔX)+(ffBb*ΔX)+(ffBc),
                                ΔY>=0 ? (ffLa*ΔX*ΔX)+(ffLb*ΔX)+(ffLc) : (ffRa*ΔX*ΔX)+(ffRb*ΔX)+(ffRc)
                        ))
                        .build());

        telemetry.addData("Status", "Sequence Completed");
        telemetry.update();
    }
}