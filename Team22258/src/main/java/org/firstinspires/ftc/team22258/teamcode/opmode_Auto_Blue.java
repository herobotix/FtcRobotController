package org.firstinspires.ftc.team22258.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team22258.pedroPathing.Constants;
import org.firstinspires.ftc.team22258.teamcode.classes.LIMELIGHT;


@Autonomous(name = "Opmode (Auto, Red) [1.2.5]", group = "Autonomous")
@Configurable
public class opmode_Auto_Blue extends LinearOpMode {
  
  boolean isBlueAlliance = true;
  
  public static class Paths {
    
    public PathChain SETUP;
    public PathChain OUTTAKE0;
    public PathChain ALIGN1;
    public PathChain INTAKE1;
    public PathChain OUTTAKE1;
    public PathChain ALIGN2;
    public PathChain INTAKE2;
    public PathChain OUTTAKE2;
    public PathChain ALIGN3;
    public PathChain INTAKE3;
    public PathChain OUTTAKE3;
    public PathChain ALIGN4;
    
    public Paths(Follower follower,boolean flipped) {
      SETUP = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(
              flipXvalue(flipped,56.500),
              8.500
            ),
            new Pose(
              flipXvalue(flipped,56.500),
              10.750
            )
          )
        )
        .setConstantHeadingInterpolation(
          Math.toRadians(180)
        )
        .build();
      
      OUTTAKE0 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(
              flipXvalue(flipped,56.500),
              10.750
            ),
            new Pose(
              flipXvalue(flipped,56.500),
              13.000
            )
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(180),
          Math.toRadians(-125.5)
        )
        .build();
      
      ALIGN1 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(
              flipXvalue(flipped,56.500),
              13.000
            ),
            new Pose(
              flipXvalue(flipped,56.500),
              36.000
            )
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(-125.5),
          Math.toRadians(180)
        )
        .build();
      
      INTAKE1 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(
              flipXvalue(flipped,56.500),
              36.000
            ),
            new Pose(
              flipXvalue(flipped,20.000),
              36.000
            )
          )
        )
        .setTangentHeadingInterpolation()
        .build();
      
      OUTTAKE1 = follower
        .pathBuilder()
        .addPath(
          new BezierCurve(
            new Pose(
              flipXvalue(flipped,20.000),
              36.000
            ),
            new Pose(
              flipXvalue(flipped,56.500),
              36.000
            ),
            new Pose(
              flipXvalue(flipped,56.500),
              13.000
            )
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(180),
          Math.toRadians(-125.5)
        )
        .build();
      
      ALIGN2 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(flipXvalue(flipped,56.500),
              13.000
            ),
            new Pose(flipXvalue(flipped,56.500),
              60.000
            )
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(-125.5),
          Math.toRadians(180)
        )
        .build();
      
      INTAKE2 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(
              flipXvalue(flipped,56.500),
              60.000
            ),
            new Pose(
              flipXvalue(flipped,20.000),
              60.000
            )
          )
        )
        .setTangentHeadingInterpolation()
        .build();
      
      OUTTAKE2 = follower
        .pathBuilder()
        .addPath(
          new BezierCurve(
            new Pose(
              flipXvalue(flipped,20.000),
              60.000
            ),
            new Pose(
              flipXvalue(flipped,56.500),
              60.000
            ),
            new Pose(
              flipXvalue(flipped,56.500),
              13.000
            )
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(180),
          Math.toRadians(-125.5)
        )
        .build();
      
      ALIGN3 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(
              flipXvalue(flipped,56.500),
              13.000),
            new Pose(
              flipXvalue(flipped,56.500),
              84.000)
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(-125.5),
          Math.toRadians(180)
        )
        .build();
      
      INTAKE3 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(flipXvalue(flipped,56.500),
              84.000),
            new Pose(flipXvalue(flipped,20.000),
              84.000)
          )
        )
        .setTangentHeadingInterpolation()
        .build();
      
      OUTTAKE3 = follower
        .pathBuilder()
        .addPath(
          new BezierCurve(
            new Pose(flipXvalue(flipped,20.000),
              84.000),
            new Pose(flipXvalue(flipped,56.500),
              84.000),
            new Pose(flipXvalue(flipped,56.500),
              13.000)
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(180),
          Math.toRadians(-125.5)
        )
        .build();
      
      ALIGN4 = follower
        .pathBuilder()
        .addPath(
          new BezierCurve(
            new Pose(flipXvalue(flipped,56.500),
              13.000),
            new Pose(flipXvalue(flipped,56.500),
              24.000),
            new Pose(flipXvalue(flipped,48.000),
              24.000)
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(-125.5),
          Math.toRadians(90)
        )
        .build();
    }
  }
  
  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pedro Pathing follower instance
  
  private Paths paths; // Paths defined in the Paths class
  
  private enum PathState {
    SETUP,
    OUTTAKE0,
    FIRE0,
    ALIGN1,
    INTAKE1,
    OUTTAKE1,
    FIRE1,
    ALIGN2,
    INTAKE2,
    OUTTAKE2,
    FIRE2,
    ALIGN3,
    INTAKE3,
    OUTTAKE3,
    FIRE3,
    ALIGN4,
    END
  }
  
  private PathState pathState; // Current autonomous path state (state machine)
  
  private Timer pathTimer/*, opmodeTimer*/;
  
  private DcMotor ItkMotor;
  private DcMotor OtkMotor;
  private Servo LServo;
  private Servo RServo;
  
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
      
      paths = new Paths(follower, isBlueAlliance); // Build paths
      pathState = PathState.SETUP;
    
    // Telemetry
      panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
      
      panelsTelemetry.debug("Status", "Initialized");
      panelsTelemetry.update(telemetry);
      
    // Timer
    
      pathTimer = new Timer();
      pathTimer.resetTimer();
      /*opmodeTimer = new Timer();
      opmodeTimer.resetTimer();*/
    
    // Map Hardware
      ItkMotor = hardwareMap.get(DcMotor.class, "ItkMotor");
      OtkMotor = hardwareMap.get(DcMotor.class, "OtkMotor");
      LServo = hardwareMap.get(Servo.class, "LServo");
      RServo = hardwareMap.get(Servo.class, "RServo");
      
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
      case SETUP:     if (isTimerSecsOver(0)) {
        setOtkServoPos(ServoState.CLOSED);
        OtkMotor.setPower(0);
        follower.followPath(paths.SETUP, true);
        setPathState(PathState.OUTTAKE0);
      } break;
      case OUTTAKE0:  if (!follower.isBusy()) {
        OtkMotor.setPower(0.67);
        ItkMotor.setPower(0);
        follower.followPath(paths.OUTTAKE0,true);
        setPathState(PathState.FIRE0);
      } break;
      case FIRE0:     if (!follower.isBusy()) {
        setOtkServoPos(ServoState.OPEN);
        setPathState(PathState.ALIGN1);
      } break;
      case ALIGN1:    if (isTimerSecsOver(2)) {
        setOtkServoPos(ServoState.CLOSED);
        OtkMotor.setPower(0);
        follower.followPath(paths.ALIGN1,true);
        setPathState(PathState.INTAKE1);
      } break;
      case INTAKE1:   if (!follower.isBusy()) {
        ItkMotor.setPower(1);
        follower.followPath(paths.INTAKE1);
        setPathState(PathState.OUTTAKE1);
      } break;
      case OUTTAKE1:  if (!follower.isBusy()) {
        OtkMotor.setPower(0.67);
        ItkMotor.setPower(0);
        follower.followPath(paths.OUTTAKE1,true);
        setPathState(PathState.FIRE1);
      } break;
      case FIRE1:     if (!follower.isBusy()) {
        setOtkServoPos(ServoState.OPEN);
        setPathState(PathState.ALIGN2);
      } break;
      case ALIGN2:    if (isTimerSecsOver(2)) {
        setOtkServoPos(ServoState.CLOSED);
        OtkMotor.setPower(0);
        follower.followPath(paths.ALIGN2,true);
        setPathState(PathState.INTAKE2);
      } break;
      case INTAKE2:   if (!follower.isBusy()) {
        ItkMotor.setPower(1);
        follower.followPath(paths.INTAKE2);
        setPathState(PathState.OUTTAKE2);
      } break;
      case OUTTAKE2:  if (!follower.isBusy()) {
        OtkMotor.setPower(0.67);
        ItkMotor.setPower(0);
        follower.followPath(paths.OUTTAKE2, true);
        setPathState(PathState.FIRE2);
      } break;
      case FIRE2:     if (!follower.isBusy()) {
        setOtkServoPos(ServoState.OPEN);
        setPathState(PathState.ALIGN3);
      } break;
      case ALIGN3:    if (isTimerSecsOver(2)) {
        setOtkServoPos(ServoState.CLOSED);
        OtkMotor.setPower(0);
        follower.followPath(paths.ALIGN3,true);
        setPathState(PathState.INTAKE3);
      } break;
      case INTAKE3:   if (!follower.isBusy()) {
        ItkMotor.setPower(1);
        follower.followPath(paths.INTAKE3);
        setPathState(PathState.OUTTAKE3);
      } break;
      case OUTTAKE3:  if (!follower.isBusy()) {
        OtkMotor.setPower(0.67);
        ItkMotor.setPower(0);
        follower.followPath(paths.OUTTAKE3, true);
        setPathState(PathState.FIRE3);
      } break;
      case FIRE3:     if (!follower.isBusy()) {
        setOtkServoPos(ServoState.OPEN);
        setPathState(PathState.ALIGN4);
      } break;
      case ALIGN4:    if (isTimerSecsOver(2)) {
        setOtkServoPos(ServoState.CLOSED);
        OtkMotor.setPower(0);
        follower.followPath(paths.ALIGN4, true);
        setPathState(PathState.END);
      } break;
    }
  }
  
  /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
  private void setPathState(PathState pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }
  
  private boolean isTimerSecsOver(int Time) {
    return pathTimer.getElapsedTimeSeconds() > Time;
  }
  
  private void setOtkServoPos(ServoState sState) {
    LServo.setPosition(
      (sState== ServoState.OPEN)?1:0
    );
    RServo.setPosition(
      (sState== ServoState.OPEN)?1:0
    );
  }
  
  private void Fn_OnStop() {
    //Run On STOP
    
    //Limelight
    Limelight.Stop();
    
  }
  
  private static double flipXvalue(boolean flipped, double x) {
    return flipped? (72 - x) : x;
  }
}