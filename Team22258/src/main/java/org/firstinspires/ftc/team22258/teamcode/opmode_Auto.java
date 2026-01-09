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


@Autonomous(name = "Opmode (Auto) [1.2.3]", group = "Autonomous")
@Configurable
public class opmode_Auto extends LinearOpMode {
  
  public static class Paths {
    
    public PathChain SetupAlign;
    public PathChain Outtake1;
    public PathChain Align1;
    public PathChain Intake1;
    public PathChain Outtake2;
    public PathChain Align2;
    public PathChain Intake2;
    public PathChain Outtake3;
    public PathChain Align3;
    public PathChain Intake3;
    public PathChain Outtake4;
    public PathChain EndAlign;
    
    public Paths(Follower follower) {
      SetupAlign = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 8.500), new Pose(56.500, 10.750))
        )
        .setConstantHeadingInterpolation(Math.toRadians(180))
        .build();
      
      Outtake1 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 10.750), new Pose(56.500, 13.000))
        )
        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-125.5))
        .build();
      
      Align1 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 13.000), new Pose(56.500, 36.000))
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(-125.5),
          Math.toRadians(180)
        )
        .build();
      
      Intake1 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 36.000), new Pose(20.000, 36.000))
        )
        .setTangentHeadingInterpolation()
        .build();
      
      Outtake2 = follower
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
          Math.toRadians(-125.5)
        )
        .build();
      
      Align2 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 13.000), new Pose(56.500, 60.000))
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(-125.5),
          Math.toRadians(180)
        )
        .build();
      
      Intake2 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 60.000), new Pose(20.000, 60.000))
        )
        .setTangentHeadingInterpolation()
        .build();
      
      Outtake3 = follower
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
          Math.toRadians(-125.5)
        )
        .build();
      
      Align3 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 13.000), new Pose(56.500, 84.000))
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(-125.5),
          Math.toRadians(180)
        )
        .build();
      
      Intake3 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(new Pose(56.500, 84.000), new Pose(20.000, 84.000))
        )
        .setTangentHeadingInterpolation()
        .build();
      
      Outtake4 = follower
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
          Math.toRadians(-125.5)
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
        .setLinearHeadingInterpolation(Math.toRadians(-125.5), Math.toRadians(90))
        .build();
    }
  }
  
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  
  private Paths paths; // Paths defined in the Paths class
  
  private enum PathState {
    SETUP,
    OUTTAKE1,
    FIRE1,
    ALIGN1,
    INTAKE1,
    OUTTAKE2,
    FIRE2,
    ALIGN2,
    INTAKE2,
    OUTTAKE3,
    FIRE3,
    ALIGN3,
    INTAKE3,
    OUTTAKE4,
    FIRE4,
    ALIGN4,
    END,
    NULL
  }
  private PathState pathState; // Current autonomous path state (state machine)
  
  private Timer pathTimer, opmodeTimer;
  
  private IMU rIMU;
  
  private DcMotor ItkMotor;
  private DcMotor OtkMotor;
  private Servo OtkServo1;
  private Servo OtkServo2;
  
  private enum ServoState {
    OPEN,
    CLOSED
  }
  
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
      pathState = PathState.SETUP;
    
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
      OtkServo1 = hardwareMap.get(Servo.class, "OtkServo1");
      OtkServo2 = hardwareMap.get(Servo.class, "OtkServo2");
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
      setOtkServoPos(ServoState.OPEN);
      
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
        /* You could check for
        - Follower State: "if(!follower.isBusy()) {}"
         // If the follower has finished its assigned task.
        - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
         // If a certain amount of time has passed since last pathState change.
        - Robot Position: "if(follower.getPose().getX() > 36) {}" //
         // If the robot is at a certain position.
        */
      case SETUP:
        follower.followPath(paths.SetupAlign);
        setPathState(PathState.OUTTAKE1);
        break;
      case OUTTAKE1:
        if(!follower.isBusy()) {
          follower.followPath(paths.Outtake1);
          OtkMotor.setPower(0.67);
          setPathState(PathState.FIRE1);
        } break;
      case FIRE1:
        if(!follower.isBusy()) {
          setOtkServoPos(ServoState.OPEN);
          setPathState(PathState.ALIGN1);
        } break;
      case ALIGN1:
        if(pathTimer.getElapsedTimeSeconds() > 2) {
          follower.followPath(paths.Align1,true);
          setOtkServoPos(ServoState.CLOSED);
          OtkMotor.setPower(0);
          setPathState(PathState.INTAKE1);
        } break;
      case INTAKE1:
        if(!follower.isBusy()) {
          follower.followPath(paths.Intake1);
          setPathState(PathState.OUTTAKE2);
        } break;
      case OUTTAKE2:
        if(!follower.isBusy()) {
          follower.followPath(paths.Outtake2,true);
          OtkMotor.setPower(0.67);
          setPathState(PathState.FIRE2);
        } break;
      case FIRE2:
        if(!follower.isBusy()) {
          setOtkServoPos(ServoState.OPEN);
          setPathState(PathState.ALIGN2);
        } break;
      case ALIGN2:
        if(pathTimer.getElapsedTimeSeconds() > 2) {
          follower.followPath(paths.Align2,true);
          setOtkServoPos(ServoState.CLOSED);
          OtkMotor.setPower(0);
          setPathState(PathState.INTAKE2);
        } break;
      case INTAKE2:
        if(!follower.isBusy()) {
          follower.followPath(paths.Intake2);
          setPathState(PathState.OUTTAKE3);
        } break;
      case OUTTAKE3:
        if(!follower.isBusy()) {
          follower.followPath(paths.Outtake3, true);
          OtkMotor.setPower(0.67);
          setPathState(PathState.FIRE3);
        } break;
      case FIRE3:
        if(!follower.isBusy()) {
          setOtkServoPos(ServoState.OPEN);
          setPathState(PathState.ALIGN3);
        } break;
      case ALIGN3:
        if(pathTimer.getElapsedTimeSeconds() > 2) {
          follower.followPath(paths.Align3,true);
          setOtkServoPos(ServoState.CLOSED);
          OtkMotor.setPower(0);
          setPathState(PathState.INTAKE3);
        } break;
      case INTAKE3:
        if(!follower.isBusy()) {
          follower.followPath(paths.Intake3);
          setPathState(PathState.OUTTAKE4);
        } break;
      case OUTTAKE4:
        if(!follower.isBusy()) {
          follower.followPath(paths.Outtake4, true);
          OtkMotor.setPower(0.67);
          setPathState(PathState.FIRE4);
        } break;
      case FIRE4:
        if(!follower.isBusy()) {
          setOtkServoPos(ServoState.OPEN);
          setPathState(PathState.ALIGN4);
        } break;
      case ALIGN4:
        if(pathTimer.getElapsedTimeSeconds() > 2) {
          follower.followPath(paths.EndAlign, true);
          setOtkServoPos(ServoState.CLOSED);
          OtkMotor.setPower(0);
          setPathState(PathState.END);
        } break;
      case END:
        if(!follower.isBusy()) {
          setPathState(PathState.NULL);
        } break;
    }
  }
  
  /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
  private void setPathState(PathState pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }
  
  private void setOtkServoPos(ServoState sState) {
    OtkServo1.setPosition(
      (sState==ServoState.OPEN)?1:0
    );
    OtkServo2.setPosition(
      (sState==ServoState.OPEN)?1:0
    );
  }
  
  private void Fn_OnStop() {
    //Run On STOP
    
    //Limelight
    Limelight.Stop();
    
  }
  
}