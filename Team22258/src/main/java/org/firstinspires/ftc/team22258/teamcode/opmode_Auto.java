package org.firstinspires.ftc.team22258.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team22258.pedroPathing.Constants;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@Autonomous(name = "Opmode (Auto) [1.2.1]", group = "Autonomous")
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
  
  private Timer pathTimer, opmodeTimer;
  
  private IMU rIMU;
  
  private DcMotor ItkMotor;
  private DcMotor OtkMotor;
  private Servo OtkServo;
  
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
    
    // init paths
      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
      
      paths = new Paths(follower); // Build paths
      pathState = 0;
    
    // Telemetry
      panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
      
      panelsTelemetry.debug("Status", "Initialized");
      panelsTelemetry.update(telemetry);
      
    // Timer
    
      pathTimer = new Timer();
      opmodeTimer = new Timer();
      opmodeTimer.resetTimer();
    
    // Map Hardware
      ItkMotor = hardwareMap.get(DcMotor.class, "ItkMotor");
      OtkMotor = hardwareMap.get(DcMotor.class, "OtkMotor");
      OtkServo = hardwareMap.get(Servo.class, "OtkServo");
      rIMU = hardwareMap.get(IMU.class, "rIMU");
    
    // Set Robot Orientation (IMU)
      IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
      );
      rIMU.initialize(parameters);
    
    // Set Behaviors
      ItkMotor.setDirection(DcMotor.Direction.REVERSE);
      OtkMotor.setDirection(DcMotor.Direction.REVERSE);
      ItkMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      OtkMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      OtkServo.setPosition(1);
      
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
    autonomousPathUpdate(); // Update autonomous state machine
    
    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.getPose().getX());
    panelsTelemetry.debug("Y", follower.getPose().getY());
    panelsTelemetry.debug("Heading", follower.getPose().getHeading());
    panelsTelemetry.update(telemetry);
  }
  
  public void autonomousPathUpdate() {
    switch (pathState) {
      case 0:
        follower.followPath(paths.SetupOuttake);
        OtkMotor.setPower(0.67);
        setPathState(1);
        break;
      case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
        if(!follower.isBusy()) {
          follower.followPath(paths.FirstAlign,true);
          OtkMotor.setPower(0);
          setPathState(2);
        }
        break;
      case 2:
        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
        if(!follower.isBusy()) {
          follower.followPath(paths.FirstIntake);
          setPathState(3);
        }
        break;
      case 3:
        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
        if(!follower.isBusy()) {
          /* Score Sample */
          /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
          follower.followPath(paths.FirstOuttake,true);
          OtkMotor.setPower(0.67);
          setPathState(4);
        }
        break;
      case 4:
        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
        if(!follower.isBusy()) {
          /* Grab Sample */
          /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
          follower.followPath(paths.SecondAlign,true);
          OtkMotor.setPower(0);
          setPathState(5);
        }
        break;
      case 5:
        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
        if(!follower.isBusy()) {
          /* Score Sample */
          /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
          follower.followPath(paths.SecondIntake);
          setPathState(6);
        }
        break;
      case 6:
        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
        if(!follower.isBusy()) {
          /* Grab Sample */
          /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
          follower.followPath(paths.SecondOuttake, true);
          OtkMotor.setPower(0.67);
          setPathState(7);
        }
        break;
      case 7:
        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
        if(!follower.isBusy()) {
          /* Grab Sample */
          /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
          follower.followPath(paths.ThirdAlign,true);
          OtkMotor.setPower(0);
          setPathState(8);
        }
        break;
      case 8:
        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
        if(!follower.isBusy()) {
          /* Score Sample */
          /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
          follower.followPath(paths.ThirdIntake);
          setPathState(9);
        }
        break;
      case 9:
        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
        if(!follower.isBusy()) {
          /* Grab Sample */
          /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
          follower.followPath(paths.ThirdOuttake, true);
          OtkMotor.setPower(0.67);
          setPathState(10);
        }
        break;
      case 10:
        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
        if(!follower.isBusy()) {
          /* Grab Sample */
          /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
          follower.followPath(paths.EndAlign, true);
          OtkMotor.setPower(0);
          setPathState(11);
        }
        break;
      case 11:
        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
        if(!follower.isBusy()) {
          /* Set the state to a Case we won't use or define, so it just stops running an new paths */
          setPathState(-1);
        }
        break;
    }
  }
  
  /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }
  
  private void Fn_OnStop() {
    //Run On STOP
    
    //Limelight
    Limelight.Stop();
    
  }
  
}