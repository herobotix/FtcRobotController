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


@Autonomous(name = "Opmode (Auto, TEST) [----]", group = "Autonomous")
@Configurable
public class opmode_Auto_TEST extends LinearOpMode {
  
  public static class Paths {
    
    public PathChain SETUP;
    
    public Paths(Follower follower,boolean flipped) {
      SETUP = follower.pathBuilder().addPath(
          new BezierLine(
            new Pose(72.000, 8.500),
            
            new Pose(72.000, 30.000)
          )
        )
        .setTangentHeadingInterpolation()
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
      
      paths = new Paths(follower, false); // Build paths
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