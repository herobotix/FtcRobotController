package org.firstinspires.ftc.team22258.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.team22258.pedroPathing.Constants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@Autonomous(name = "Opmode (Auto) [1.2.0]", group = "Autonomous")
@Configurable
public class opmode_Auto extends LinearOpMode {
  
  public static class Paths {
    
    public PathChain SetupOuttake;
    public PathChain FirstAlign;
    public PathChain FirstIntake;
    public PathChain FirstOuttake;
    public PathChain SecondAlign;
    public PathChain SecondIntake;
    public PathChain SecondOuttake;
    public PathChain ThirdAlign;
    public PathChain ThirdIntake;
    public PathChain ThirdOuttake;
    public PathChain EndAlign;
    
    public Paths(Follower follower) {
      SetupOuttake = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 8.500), new Pose(56.500, 13.000))
        )
        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(54.5))
        .build();
      
      FirstAlign = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 13.000), new Pose(56.500, 36.000))
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(54.5),
          Math.toRadians(180)
        )
        .build();
      
      FirstIntake = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 36.000), new Pose(20.000, 36.000))
        )
        .setTangentHeadingInterpolation()
        .build();
      
      FirstOuttake = follower
        .pathBuilder()
        .addPath(
          new BezierCurve(
            new Pose(20.000, 36.000),
            new Pose(56.500, 36.000),
            new Pose(56.500, 13.000)
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(180),
          Math.toRadians(54.5)
        )
        .build();
      
      SecondAlign = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 13.000), new Pose(56.500, 60.000))
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(54.5),
          Math.toRadians(180)
        )
        .build();
      
      SecondIntake = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 60.000), new Pose(20.000, 60.000))
        )
        .setTangentHeadingInterpolation()
        .build();
      
      SecondOuttake = follower
        .pathBuilder()
        .addPath(
          new BezierCurve(
            new Pose(20.000, 60.000),
            new Pose(56.500, 60.000),
            new Pose(56.500, 13.000)
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(180),
          Math.toRadians(54.5)
        )
        .build();
      
      ThirdAlign = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 13.000), new Pose(56.500, 84.000))
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(54.5),
          Math.toRadians(180)
        )
        .build();
      
      ThirdIntake = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 84.000), new Pose(20.000, 84.000))
        )
        .setTangentHeadingInterpolation()
        .build();
      
      ThirdOuttake = follower
        .pathBuilder()
        .addPath(
          new BezierCurve(
            new Pose(20.000, 84.000),
            new Pose(56.500, 84.000),
            new Pose(56.500, 13.000)
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(180),
          Math.toRadians(54.5)
        )
        .build();
      
      EndAlign = follower
        .pathBuilder()
        .addPath(
          new BezierCurve(
            new Pose(56.500, 13.000),
            new Pose(56.500, 24.000),
            new Pose(48.000, 24.000)
          )
        )
        .setLinearHeadingInterpolation(Math.toRadians(54.5), Math.toRadians(90))
        .build();
    }
  }
  
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  private int pathState; // Current autonomous path state (state machine)
  private Paths paths; // Paths defined in the Paths class
  
  private IMU rIMU;
  
  private LIMELIGHT Limelight;
  
  @Override
  public void runOpMode() {
    //Begin
    
    //Init & Wait
    Fn_Init();
    waitForStart();
    Fn_OnStart();
    
    //Run Opmode
    while (opModeIsActive()) { Fn_loop(); }
    Fn_OnStop();
    
  }
  
  private void Fn_Init() {
    // Initialization Code
    
    // ???
      panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
      
      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
      
      paths = new Paths(follower); // Build paths
      
      panelsTelemetry.debug("Status", "Initialized");
      panelsTelemetry.update(telemetry);
    
    // Map Hardware
      rIMU = hardwareMap.get(IMU.class, "rIMU");
    
    // Set Robot Orientation (IMU)
      IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
      );
      rIMU.initialize(parameters);
    
    // Limelight
      Limelight = new LIMELIGHT();
      Limelight.Init(hardwareMap);
    
  }
  
  private void Fn_OnStart() {
    // Run On START
    
    // Limelight
      Limelight.Start();
      
  }
  
  public void Fn_loop() {
    follower.update(); // Update Pedro Pathing
    pathState = autonomousPathUpdate(); // Update autonomous state machine
    
    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", follower.getPose().getHeading());
    panelsTelemetry.update(telemetry);
  }
  
  public int autonomousPathUpdate() {
    // Add your state machine Here
    // Access paths with paths.pathName
    // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
    return pathState;
  }
  
  private void Fn_OnStop() {
    //Run On STOP
    
    //Limelight
    Limelight.Stop();
    
  }
  
}