package org.firstinspires.ftc.team00000.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team00000.roadrunner.MecanumDrive;

@Config
@Autonomous(name="AutoOpTest", group="autonomous")
//@Disabled
public class AutoOpTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Wait for the game to start (driver presses PLAY)
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        timer.reset();

        // Run until the end of the match (driver presses STOP)
        if (isStopRequested()) return;

        // Using TrajectoryActionBuilder to create a sequence of actions
        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(startPose);

        // Build your sequence of actions
        /*
        Action sequence = actionBuilder
                .strafeTo(new Pose2d(-36, -60, Math.toRadians(90))) // Move to a new x position
                .setTangent(Math.toRadians(0)) // Set the tangent for the next movement
                .splineTo(new Pose2d(-36, -36, Math.toRadians(0))) // Spline to a point
                .strafeTo(new Pose2d(-36, -12, Math.toRadians(0))) // Strafe
                .waitSeconds(1) // Wait for 1 second
                // Here you could add custom actions like moving an arm or opening a claw
                // Example:
                .addTemporalMarker(0, () -> {
                    // Perform some action at the start of the trajectory, like opening a claw
                    // This marker is added at time 0, meaning at the start of this segment
                })
                .build();
        */

        // Execute the built sequence
        //drive.followAction(sequence);

        // You can continue to build more complex sequences or handle parallel actions
        telemetry.addData("Status", "Sequence Completed");
        telemetry.update();
    }
}