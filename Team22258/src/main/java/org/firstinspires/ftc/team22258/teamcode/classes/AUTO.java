package org.firstinspires.ftc.team22258.teamcode.classes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team22258.pedroPathing.Constants;

/**Version 1.2.17*/
@Configurable
public class AUTO {
  //Autonomous Code
  
  // Alliance Definition
    boolean isRedAlliance;
    Telemetry telemetry;
    public  AUTO(Telemetry telemetry, boolean isRedAlliance)      {
      this.isRedAlliance = isRedAlliance;
      this.telemetry = telemetry;
    }
    
  // Class Definitions
    private IOTAKE IOtake;
    
  // Path Definitions
    public  static class Paths                                    {
      
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
            Math.toRadians(flipXDegrees(isRedAlliance,-70))
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
            Math.toRadians(flipXDegrees(isRedAlliance,-70)),
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
            Math.toRadians(flipXDegrees(isRedAlliance,-70))
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
            Math.toRadians(flipXDegrees(isRedAlliance,-70)),
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
            Math.toRadians(flipXDegrees(isRedAlliance,-70))
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
            Math.toRadians(flipXDegrees(isRedAlliance,-70)),
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
            Math.toRadians(flipXDegrees(isRedAlliance,-70))
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
            Math.toRadians(flipXDegrees(isRedAlliance,-70)),
            Math.toRadians(flipXDegrees(isRedAlliance,90))
          )
          .build();
      }
    }
    private enum PathState                                        {
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
    private AUTO.Paths paths;
    private PathState pathState;
    private Timer pathTimer;
    
  // Follower Definition
    public  Follower follower;
    
  // Telemetry Definition
    private TelemetryManager panelsTelemetry;
    
  // Primary Functions
    public  void Init(HardwareMap hardwareMap)                    {
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
    public  void doLoop()                                         {
      follower.update(); // Update Pedro Pathing
      autonomousPathUpdate(); // Update autonomous state machine
      
      doTelemetry(telemetry);
    }
    public  void doTelemetry(Telemetry telemetry)                 {
      
      // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
    
  // Secondary Functions
    public  void autonomousPathUpdate()                           {
      switch (pathState) {
        case SETUP:     if (isTimerSecsOver(0)) {
          IOtake.runIntake(1);
          IOtake.runFlywheel(
            IOTAKE.FlywheelState .MAX
          );
          follower.followPath(paths .SETUP, true);
          setPathState(PathState.OUTTAKE0 );
        } break;
        case OUTTAKE0:  if (!follower.isBusy()) {
          follower.followPath(paths .OUTTAKE0,true);
          setPathState(PathState.FIRE0 );
        } break;
        case FIRE0:     if (!follower.isBusy()) {
          IOtake.runOtkGate(
            IOTAKE.OuttakeServoState .OPEN
          );
          setPathState(PathState.ALIGN1 );
        } break;
        case ALIGN1:    if (isTimerSecsOver(3)) {
          IOtake.runOtkGate(
            IOTAKE.OuttakeServoState .CLOSED
          );
          follower.followPath(paths .ALIGN1,true);
          setPathState(PathState.INTAKE1 );
        } break;
        case INTAKE1:   if (!follower.isBusy()) {
          follower.followPath(paths .INTAKE1, true);
          setPathState(PathState.OUTTAKE1 );
        } break;
        case OUTTAKE1:  if (!follower.isBusy()) {
          follower.followPath(paths .OUTTAKE1,true);
          setPathState(PathState.FIRE1 );
        } break;
        case FIRE1:     if (!follower.isBusy()) {
          IOtake.runOtkGate(
            IOTAKE.OuttakeServoState .OPEN
          );
          setPathState(PathState.ALIGN2 );
        } break;
        case ALIGN2:    if (isTimerSecsOver(3)) {
          IOtake.runOtkGate(
            IOTAKE.OuttakeServoState .CLOSED
          );
          follower.followPath(paths .ALIGN2,true);
          setPathState(PathState.INTAKE2 );
        } break;
        case INTAKE2:   if (!follower.isBusy()) {
          follower.followPath(paths .INTAKE2, true);
          setPathState(PathState.OUTTAKE2 );
        } break;
        case OUTTAKE2:  if (!follower.isBusy()) {
          follower.followPath(paths .OUTTAKE2, true);
          setPathState(PathState.FIRE2 );
        } break;
        case FIRE2:     if (!follower.isBusy()) {
          IOtake.runOtkGate(
            IOTAKE.OuttakeServoState .OPEN
          );
          setPathState(PathState.ALIGN3 );
        } break;
        case ALIGN3:    if (isTimerSecsOver(3)) {
          IOtake.runOtkGate(
            IOTAKE.OuttakeServoState .CLOSED
          );
          follower.followPath(paths .ALIGN3,true);
          setPathState(PathState.INTAKE3 );
        } break;
        case INTAKE3:   if (!follower.isBusy()) {
          follower.followPath(paths .INTAKE3, true);
          setPathState(PathState.OUTTAKE3 );
        } break;
        case OUTTAKE3:  if (!follower.isBusy()) {
          follower.followPath(paths .OUTTAKE3, true);
          setPathState(PathState.FIRE3 );
        } break;
        case FIRE3:     if (!follower.isBusy()) {
          IOtake.runOtkGate(
            IOTAKE.OuttakeServoState .OPEN
          );
          setPathState(PathState.ALIGN4 );
        } break;
        case ALIGN4:    if (isTimerSecsOver(3)) {
        IOtake.runIntake(0);
        IOtake.runOtkGate(
          IOTAKE.OuttakeServoState .CLOSED
        );
        IOtake.runFlywheel(
          IOTAKE.FlywheelState .OFF
        );
        follower.followPath(paths .ALIGN4, true);
        setPathState(PathState.END );
      } break;
      }
    }
    private boolean isTimerSecsOver(int Time)                     {
      return pathTimer.getElapsedTimeSeconds() > Time;
      
    }
    
  // Variable Functions
    private void setPathState(PathState pState)                   {
      pathState = pState;
      pathTimer.resetTimer();
    }
    
  // Other Functions
    private static double flipXValue(boolean flipped, double x)   {
      return flipped? (72 - x) : x;
      
    }
    private static double flipXDegrees(boolean flipped, double x) {
      return flipped? (180 - x) : x;
      
    }
    
}
