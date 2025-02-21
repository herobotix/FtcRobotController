package org.firstinspires.ftc.team22258.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.team22258.roadrunner.MecanumDrive;

@Config
@Autonomous(name = "AutoMain", group="autonomous")
public final class AutoMain extends LinearOpMode {
    public static double
        //fudge factors
        ffFa = 0.9060, ffFb = 1.0080,
        ffBa = 0.9103, ffBb = 1.0060,
        ffLa = 2.0240, ffLb = 0.8703,
        ffRa = 1.6260, ffRb = 0.9174,
    
        // Start Position
        startX = 0,     //127,
        startY = 0,    //80.5,
        startH = 0, //-Math.PI,
    
        deltaX = 0,
        deltaY = 0,
        dT = 1.0, //sleep time for claw change
    
        // Current Position
        currentX = startX,
        currentY = startY,
        
        // Chamber Position
        ChamberX = 20,      //107,
        ChamberY = 7.5,    //73,
        
        // upX (pre-mark X) & nearY (Observation Zone Y) Positions
        upY= 48,
        nearX = -59,
        
        //Sample Marks' Positions
        farX = -16,
        mark1X= 54,
        mark2X= 65,
        mark3X= 77
    ;
    
    public static int
        //Arm Constants
        targetArmPos = 3600
    ;
    
    public boolean hR = false;
    
    public double YawReverse(double angle) {
        hR = hR ? false : true;
        return angle;
    }
    
    public Vector2d StrafeTuna(double dX, double dY) {
        double sn = hR ? -1 : 1;
        double ffdX = (sn*dX>=0)?(sn*ffFa*Math.pow(sn*dX,ffFb)):(-sn*ffBa*Math.pow(-sn*dX,ffBb));
        double ffdY = (sn*dY>=0)?(sn*ffLa*Math.pow(sn*dY,ffLb)):(-sn*ffRa*Math.pow(-sn*dY,ffRb));
        Vector2d newPos = new Vector2d(currentX + ffdX, currentY + ffdY);
        currentX += dX; currentY += dY;
        return newPos;
    }
    
    @Override
    public void runOpMode() throws InterruptedException {
      
        // Setup Robot Position
        Pose2d beginPose1 = new Pose2d(startX, startY, startH);
        Pose2d beginPose2 = new Pose2d(startX, startY, startH+Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose1);
        TrajectoryActionBuilder driveAB1 = drive.actionBuilder(beginPose1);
        TrajectoryActionBuilder driveAB2 = drive.actionBuilder(beginPose2);
        ArmTool arm = new ArmTool(hardwareMap,"Arm");
        IMU imu = hardwareMap.get(IMU.class, "rIMU");
        
        waitForStart();
        
        // Create & Run Actions
        Actions.runBlocking(
            new SequentialAction(
                //driveAB1.strafeTo(StrafeTune(startX+deltaX,startY+deltaY)).build(),
                arm.ClawState(false),
                new SleepAction(dT),
                driveAB1
                    .strafeTo(StrafeTuna(5,0))
                    .strafeTo(StrafeTuna(0,7.5))
                    .turn(YawReverse(Math.PI))
                    .build(),
                driveAB2
                    .strafeTo(StrafeTuna(17,0))
                    .build(),
                arm.ArmTo(targetArmPos),
                new SleepAction(dT/2),
                driveAB2
                    .strafeTo(StrafeTuna(-20,0))
                    .strafeTo(StrafeTuna(3,0))
                    .build(),
                arm.ClawState(true),
                new SleepAction(dT/1.5),
                arm.ArmTo(0),
                driveAB2
                    .strafeTo(StrafeTuna(0, -25))
                    .strafeTo(StrafeTuna(40, 0))
                    .strafeTo(StrafeTuna(0, -9))
                    .strafeTo(StrafeTuna(-45, 0))
                    .strafeTo(StrafeTuna(45, 0))
                    .strafeTo(StrafeTuna(0, -9))
                    .strafeTo(StrafeTuna(-45, 0))
                    .strafeTo(StrafeTuna(45, 0))
                    .strafeTo(StrafeTuna(0, -9))
                    .strafeTo(StrafeTuna(-45, 0))
                    .build()
            )
        );
    }
}
