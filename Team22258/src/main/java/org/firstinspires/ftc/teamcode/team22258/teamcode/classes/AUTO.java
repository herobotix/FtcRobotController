package org.firstinspires.ftc.teamcode.team22258.teamcode.classes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team22258.pedroPathing.Constants;

import java.util.Objects;

@Configurable
abstract public class AUTO extends LinearOpMode {
  //Autonomous Code
  
  // Version Number Definition
  public static final String Version
    = "1.2.22";
  
  // Alliance Definition
  protected boolean isRedAlliance;
  
  // Class Definition
  private IOTAKE IOtake;
  
  // Path Definitions
  public static double FireAngleDegreesBlue = 297;
  public static double FireAngleDegreesRed  = 240;
  public static class Paths                                          {
    
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
    
    public Paths(Follower follower, boolean isRedAlliance) {
      SETUP = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(
              flipXValue(isRedAlliance,56.500),
              8.500
            ),
            new Pose(
              flipXValue(isRedAlliance,56.500),
              10.750
            )
          )
        )
        .setConstantHeadingInterpolation(
          Math.toRadians(flipXDegrees(isRedAlliance,180))
        )
        .build();
      
      OUTTAKE0 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(
              flipXValue(isRedAlliance,56.500),
              10.750
            ),
            new Pose(
              flipXValue(isRedAlliance,56.500),
              13.000
            )
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(flipXDegrees(isRedAlliance,180)),
          Math.toRadians(isRedAlliance?(FireAngleDegreesRed):(FireAngleDegreesBlue))
        )
        .build();
      
      ALIGN1 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(
              flipXValue(isRedAlliance,56.500),
              13.000
            ),
            new Pose(
              flipXValue(isRedAlliance,56.500),
              36.000
            )
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(isRedAlliance?(FireAngleDegreesRed):(FireAngleDegreesBlue)),
          Math.toRadians(flipXDegrees(isRedAlliance,180))
        )
        .build();
      
      INTAKE1 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(
              flipXValue(isRedAlliance,56.500),
              36.000
            ),
            new Pose(
              flipXValue(isRedAlliance,20.000),
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
              flipXValue(isRedAlliance,20.000),
              36.000
            ),
            new Pose(
              flipXValue(isRedAlliance,56.500),
              36.000
            ),
            new Pose(
              flipXValue(isRedAlliance,56.500),
              13.000
            )
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(flipXDegrees(isRedAlliance,180)),
          Math.toRadians(isRedAlliance?(FireAngleDegreesRed):(FireAngleDegreesBlue))
        )
        .build();
      
      ALIGN2 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(flipXValue(isRedAlliance,56.500),
              13.000
            ),
            new Pose(flipXValue(isRedAlliance,56.500),
              60.000
            )
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(isRedAlliance?(FireAngleDegreesRed):(FireAngleDegreesBlue)),
          Math.toRadians(flipXDegrees(isRedAlliance,180))
        )
        .build();
      
      INTAKE2 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(
              flipXValue(isRedAlliance,56.500),
              60.000
            ),
            new Pose(
              flipXValue(isRedAlliance,20.000),
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
              flipXValue(isRedAlliance,20.000),
              60.000
            ),
            new Pose(
              flipXValue(isRedAlliance,56.500),
              60.000
            ),
            new Pose(
              flipXValue(isRedAlliance,56.500),
              13.000
            )
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(flipXDegrees(isRedAlliance,180)),
          Math.toRadians(isRedAlliance?(FireAngleDegreesRed):(FireAngleDegreesBlue))
        )
        .build();
      
      ALIGN3 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(
              flipXValue(isRedAlliance,56.500),
              13.000),
            new Pose(
              flipXValue(isRedAlliance,56.500),
              84.000)
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(isRedAlliance?(FireAngleDegreesRed):(FireAngleDegreesBlue)),
          Math.toRadians(flipXDegrees(isRedAlliance,180))
        )
        .build();
      
      INTAKE3 = follower
        .pathBuilder()
        .addPath(
          new BezierLine(
            new Pose(flipXValue(isRedAlliance,56.500),
              84.000),
            new Pose(flipXValue(isRedAlliance,20.000),
              84.000)
          )
        )
        .setTangentHeadingInterpolation()
        .build();
      
      OUTTAKE3 = follower
        .pathBuilder()
        .addPath(
          new BezierCurve(
            new Pose(flipXValue(isRedAlliance,20.000),
              84.000),
            new Pose(flipXValue(isRedAlliance,56.500),
              84.000),
            new Pose(flipXValue(isRedAlliance,56.500),
              13.000)
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(flipXDegrees(isRedAlliance,180)),
          Math.toRadians(isRedAlliance?(FireAngleDegreesRed):(FireAngleDegreesBlue))
        )
        .build();
      
      ALIGN4 = follower
        .pathBuilder()
        .addPath(
          new BezierCurve(
            new Pose(flipXValue(isRedAlliance,56.500),
              13.000),
            new Pose(flipXValue(isRedAlliance,56.500),
              24.000),
            new Pose(flipXValue(isRedAlliance,48.000),
              24.000)
          )
        )
        .setLinearHeadingInterpolation(
          Math.toRadians(isRedAlliance?(FireAngleDegreesRed):(FireAngleDegreesBlue)),
          Math.toRadians(flipXDegrees(isRedAlliance,90))
        )
        .build();
    }
  }
  private enum PathType                                              {
    SETUP,
    ALIGN,
    INTAKE,
    OUTTAKE,
    FIRE,
    END
  }
  private enum PathState                                             {
    SETUP   (PathType .SETUP   ),
    OUTTAKE0(PathType .OUTTAKE ),
    FIRE0   (PathType .FIRE    ),
    ALIGN1  (PathType .ALIGN   ),
    INTAKE1 (PathType .INTAKE  ),
    OUTTAKE1(PathType .OUTTAKE ),
    FIRE1   (PathType .FIRE    ),
    ALIGN2  (PathType .ALIGN   ),
    INTAKE2 (PathType .INTAKE  ),
    OUTTAKE2(PathType .OUTTAKE ),
    FIRE2   (PathType .FIRE    ),
    ALIGN3  (PathType .ALIGN   ),
    INTAKE3 (PathType .INTAKE  ),
    OUTTAKE3(PathType .OUTTAKE ),
    FIRE3   (PathType .FIRE    ),
    ALIGN4  (PathType .ALIGN   ),
    END     (PathType .END     );
    
    private final PathType type;
    public PathType getType()           {
      return type;
      
    }
    PathState(PathType type) {
      this.type  = type;
    }
    
  }
  private Paths paths;
  private PathState pathState;
  private Timer pathTimer;
  private byte fireNum = 0;
  
  // Follower Definition
  public  Follower follower;
  
  // Telemetry Definition
  private TelemetryManager panelsTelemetry;
  
  // Run Function
  @Override
  public void runOpMode                  ()                          {
    //Begin
    
    // Init & Wait
    Init();
    waitForStart();
    
    // Run Opmode
    while (opModeIsActive()) { doLoop(); }
    
  }
  
  // Primary Functions
  public  void Init                      ()                          {
    //Initialization Code
    
    //Classes
    IOtake = new IOTAKE();
    IOtake.Init(hardwareMap);
    
    // Telemetry
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    
    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);
    
    // Init Follower
    follower = Constants.createFollower(hardwareMap);
    follower.setStartingPose(new Pose(
      flipXValue(isRedAlliance,56.500),
      8.500,
      Math.toRadians(flipXDegrees(isRedAlliance,180))
    ));
    
    // Build Paths
    paths = new AUTO.Paths(follower, isRedAlliance);
    pathState = PathState.SETUP ;
    
    // Timer
    pathTimer = new Timer();
    pathTimer .resetTimer();
    
  }
  public  void doLoop                    ()                          {
    //Loop Code
    
    // Update Paths
    follower .update      ();
    autonomousPathUpdate  ();
    
    // Update Telemetry
    doTelemetry           ();
    
  }
  public  void doTelemetry               ()                          {
    //Telemetry Code
    
    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.getPose() .getX() );
    panelsTelemetry.debug("Y", follower.getPose() .getY() );
    panelsTelemetry.debug("Heading", follower.getPose() .getHeading() );
    panelsTelemetry.debug("Fire #", fireNum);
    panelsTelemetry.debug("Timer", pathTimer .getElapsedTimeSeconds() );
    IOtake.doTelemetry(panelsTelemetry);
    panelsTelemetry.update(telemetry);
  }
  
  // Secondary Functions
  private void      autonomousPathUpdate ()                          {
    switch (pathState.getType()) {
      case SETUP:                           {
        //Setup Code
        
        // Path Control
        follower.followPath(getPath(), true);
        nextPathState();
        
      } break;
      case OUTTAKE: if (!follower.isBusy()) {
        //Prep For Firing
        
        // Activate Flywheel
        IOtake.runFlywheel(IOTAKE.FlywheelState.BIG);
        
        // Path Control
        follower.followPath(getPath(), true);
        nextPathState();
        
      } break;
      case FIRE:    if (!follower.isBusy()) {
        //Firing Code
        
        doFiring();
      } break;
      case ALIGN:                           {
        //Align Code
        
        // Close gate and stop both Flywheel and Intake
        IOtake.runOtkGate(IOTAKE.OuttakeServoState.CLOSED);
        IOtake.runFlywheel(IOTAKE.FlywheelState.OFF);
        IOtake.runIntake(0 );
        
        // Path Control
        follower.followPath(getPath(),true);
        nextPathState();
        
      } break;
      case INTAKE:  if (!follower.isBusy()) {
        //Intake Code
        
        // Activate Intake
        IOtake.runIntake(1 );
        
        // Path Control
        follower.followPath(getPath(),true);
        nextPathState();
        
      } break;
      case END:                             {
        //End Code
        break;
      }
    }
  }
  private PathChain getPath              ()                          {
    switch (pathState) {
      case SETUP:     return paths .SETUP     ;
      case OUTTAKE0:  return paths .OUTTAKE0  ;
      case ALIGN1:    return paths .ALIGN1    ;
      case INTAKE1:   return paths .INTAKE1   ;
      case OUTTAKE1:  return paths .OUTTAKE1  ;
      case ALIGN2:    return paths .ALIGN2    ;
      case INTAKE2:   return paths .INTAKE2   ;
      case OUTTAKE2:  return paths .OUTTAKE2  ;
      case ALIGN3:    return paths .ALIGN3    ;
      case INTAKE3:   return paths .INTAKE3   ;
      case OUTTAKE3:  return paths .OUTTAKE3  ;
      case ALIGN4:    return paths .ALIGN4    ;
    }
    return null;
  }
  private void      doFiring             ()                          {
    if (pathTimer.getElapsedTimeSeconds() > .4) {
      if (
        (fireNum == 0) ||
          (fireNum == 2) ||
          (fireNum == 4)
      ) {
        IOtake.runOtkGate(IOTAKE.OuttakeServoState.OPEN);
        IOtake.runIntake(1);
        fireNum++;
      }
      if (
        (pathTimer.getElapsedTimeSeconds() > .8) &&
          (
            (fireNum == 1) ||
              (fireNum == 3) ||
              (fireNum == 5)
          )
      ) {
        pathTimer.resetTimer();
        IOtake.runOtkGate(IOTAKE.OuttakeServoState.CLOSED);
        IOtake.runIntake(0);
        fireNum++;
        if (fireNum == 6) {
          nextPathState();
          fireNum = 0;
        }
      }
    }
  }
  
  // Variable Functions
  private void nextPathState             ()                          {
    if (Objects .requireNonNull(pathState) != PathState .END ) {
      pathTimer .resetTimer();
    }
    switch (pathState)                                         {
      case SETUP    : pathState = PathState .OUTTAKE0 ; break ;
      case OUTTAKE0 : pathState = PathState .FIRE0    ; break ;
      case FIRE0    : pathState = PathState .ALIGN1   ; break ;
      case ALIGN1   : pathState = PathState .INTAKE1  ; break ;
      case INTAKE1  : pathState = PathState .OUTTAKE1 ; break ;
      case OUTTAKE1 : pathState = PathState .FIRE1    ; break ;
      case FIRE1    : pathState = PathState .ALIGN2   ; break ;
      case ALIGN2   : pathState = PathState .INTAKE2  ; break ;
      case INTAKE2  : pathState = PathState .OUTTAKE2 ; break ;
      case OUTTAKE2 : pathState = PathState .FIRE2    ; break ;
      case FIRE2    : pathState = PathState .ALIGN3   ; break ;
      case ALIGN3   : pathState = PathState .INTAKE3  ; break ;
      case INTAKE3  : pathState = PathState .OUTTAKE3 ; break ;
      case OUTTAKE3 : pathState = PathState .FIRE3    ; break ;
      case FIRE3    : pathState = PathState .ALIGN4   ; break ;
      case ALIGN4   : pathState = PathState .END      ; break ;
    }
  }
  
  // Other Functions
  private static double flipXValue       (boolean flipped, double x) {
    return flipped? (72 - x) : x;
    
  }
  private static double flipXDegrees     (boolean flipped, double x) {
    return flipped? (180 - x) : x;
    
  }
  
}
