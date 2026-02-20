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

/**Version 1.2.19*/
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
    public static double FireAngleDegrees = -63;
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
            Math.toRadians(flipXDegrees(isRedAlliance, FireAngleDegrees))
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
            Math.toRadians(flipXDegrees(isRedAlliance, FireAngleDegrees)),
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
            Math.toRadians(flipXDegrees(isRedAlliance, FireAngleDegrees))
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
            Math.toRadians(flipXDegrees(isRedAlliance, FireAngleDegrees)),
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
            Math.toRadians(flipXDegrees(isRedAlliance, FireAngleDegrees))
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
            Math.toRadians(flipXDegrees(isRedAlliance, FireAngleDegrees)),
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
            Math.toRadians(flipXDegrees(isRedAlliance, FireAngleDegrees))
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
            Math.toRadians(flipXDegrees(isRedAlliance, FireAngleDegrees)),
            Math.toRadians(flipXDegrees(isRedAlliance,90))
          )
          .build();
      }
    }
    private enum PathType                                         {
      SETUP,
      ALIGN,
      INTAKE,
      OUTTAKE,
      FIRE,
      END
    }
    private enum PathState                                        {
      SETUP   (PathType .SETUP   , 0),
      OUTTAKE0(PathType .OUTTAKE , 0),
      FIRE0   (PathType .FIRE    , 0),
      ALIGN1  (PathType .ALIGN   , 1),
      INTAKE1 (PathType .INTAKE  , 1),
      OUTTAKE1(PathType .OUTTAKE , 1),
      FIRE1   (PathType .FIRE    , 1),
      ALIGN2  (PathType .ALIGN   , 2),
      INTAKE2 (PathType .INTAKE  , 2),
      OUTTAKE2(PathType .OUTTAKE , 2),
      FIRE2   (PathType .FIRE    , 2),
      ALIGN3  (PathType .ALIGN   , 3),
      INTAKE3 (PathType .INTAKE  , 3),
      OUTTAKE3(PathType .OUTTAKE , 3),
      FIRE3   (PathType .FIRE    , 3),
      ALIGN4  (PathType .ALIGN   , 4),
      END     (PathType .END     , 0);
      
      private final int Value;
      private final PathType type;
      public int getValue()               {
        return Value;
        
      }
      public PathType getType()           {
        return type;
        
      }
      PathState(PathType type, int Value) {
        this.Value = Value;
        this.type  = type;
      }
      
    }
    private AUTO.Paths paths;
    private PathState pathState;
    private Timer pathTimer;
    private byte fireNum = 0;
    public static double
      fireTime  = .4,
      PauseTime = .4;
    
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
      //Telemetry Code
      
      // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose() .getX() );
        panelsTelemetry.debug("Y", follower.getPose() .getY() );
        panelsTelemetry.debug("Heading", follower.getPose() .getHeading() );
        panelsTelemetry.debug("Fire #", fireNum);
        panelsTelemetry.debug("Timer", pathTimer .getElapsedTimeSeconds() );
        panelsTelemetry.update(telemetry);
    }
    
  // Secondary Functions
    public  void autonomousPathUpdate()                           {
      switch (pathState) {
        case SETUP:                             {
          follower.followPath(paths.SETUP, true);
          doSetup();
        } break;
        case OUTTAKE0:  if (!follower.isBusy()) {
          follower.followPath(paths .OUTTAKE0,true);
          doFirePrep(0 );
        } break;
        case FIRE0:     if (!follower.isBusy()) {
          doFiring(1 );
        } break;
        case ALIGN1:                            {
          follower.followPath(paths .ALIGN1,true);
          doAlign(1 );
        } break;
        case INTAKE1:   if (!follower.isBusy()) {
          follower.followPath(paths .INTAKE1, true);
          doIntake(1 );
        } break;
        case OUTTAKE1:  if (!follower.isBusy()) {
          follower.followPath(paths .OUTTAKE1,true);
          doFirePrep(1 );
        } break;
        case FIRE1:     if (!follower.isBusy()) {
          doFiring(2 );
        } break;
        case ALIGN2:                            {
          follower.followPath(paths .ALIGN2,true);
          doAlign(2 );
        } break;
        case INTAKE2:   if (!follower.isBusy()) {
          follower.followPath(paths .INTAKE2, true);
          doIntake(2 );
        } break;
        case OUTTAKE2:  if (!follower.isBusy()) {
          follower.followPath(paths .OUTTAKE2, true);
          doFirePrep(2 );
        } break;
        case FIRE2:     if (!follower.isBusy()) {
          doFiring(3 );
        } break;
        case ALIGN3:                            {
          follower.followPath(paths .ALIGN3,true);
          doAlign(3 );
        } break;
        case INTAKE3:   if (!follower.isBusy()) {
          follower.followPath(paths .INTAKE3, true);
          doIntake(3 );
        } break;
        case OUTTAKE3:  if (!follower.isBusy()) {
          follower.followPath(paths .OUTTAKE3, true);
          doFirePrep(3 );
        } break;
        case FIRE3:     if (!follower.isBusy()) {
          doFiring(4 );
        } break;
        case ALIGN4:                            {
          follower.followPath(paths .ALIGN4, true);
          doEnd();
        } break;
      }
    }
    private PathState getPathState(PathType type, int Value)      {
      switch (type) {
        case ALIGN:   switch (Value)    {
          case 1: return PathState .ALIGN1 ;
          case 2: return PathState .ALIGN2 ;
          case 3: return PathState .ALIGN3 ;
          case 4: return PathState .ALIGN4 ;
        }
        case INTAKE:  switch (Value)    {
          case 1: return PathState .INTAKE1 ;
          case 2: return PathState .INTAKE2 ;
          case 3: return PathState .INTAKE3 ;
        }
        case OUTTAKE: switch (Value)    {
          case 0: return PathState .OUTTAKE0 ;
          case 1: return PathState .OUTTAKE1 ;
          case 2: return PathState .OUTTAKE2 ;
          case 3: return PathState .OUTTAKE3 ;
        }
        case FIRE:    switch (Value)    {
          case 0: return PathState .FIRE0 ;
          case 1: return PathState .FIRE1 ;
          case 2: return PathState .FIRE2 ;
          case 3: return PathState .FIRE3 ;
        }
        case SETUP:   return PathState  .SETUP ;
        default:      return PathState  .END   ;
      }
    }
    private void doSetup()                                        {
      // Setup Code
        setPathState(PathState.OUTTAKE0 );
    }
    private void doAlign(int IntakeNum)                           {
      // Align Code
        IOtake.runIntake(0);
        IOtake.runFlywheel(IOTAKE.FlywheelState .OFF );
        IOtake.runOtkGate(IOTAKE.OuttakeServoState .CLOSED );
        setPathState(
          getPathState(PathType.INTAKE, IntakeNum)
        );
    }
    private void doIntake(int OuttakeNum)                         {
      // Intake Code
        IOtake.runIntake(1);
        setPathState(
          getPathState(PathType.OUTTAKE, OuttakeNum)
        );
    }
    private void doFirePrep(int FireNum)                          {
      // Prep For Firing
        IOtake.runFlywheel( IOTAKE.FlywheelState .BIG );
        setPathState(
          getPathState(PathType.FIRE, FireNum)
        );
    }
    private void doFiring(int AlignNum)                           {
      //Prep For Firing
      
      if (pathTimer.getElapsedTimeSeconds() > .4 ) {
        if (
          ( fireNum == 0 ) ||
          ( fireNum == 2 ) ||
          ( fireNum == 4 )
        ) {
          IOtake.runOtkGate(IOTAKE.OuttakeServoState.OPEN);
          IOtake.runIntake(1);
          fireNum++;
        }
        if (
          ( pathTimer.getElapsedTimeSeconds() > .8 ) &&
          (
            ( fireNum == 1 ) ||
            ( fireNum == 3 ) ||
            ( fireNum == 5 )
          )
        ) {
          pathTimer.resetTimer();
          IOtake.runOtkGate(IOTAKE.OuttakeServoState .CLOSED );
          IOtake.runIntake(0);
          fireNum++;
          if (fireNum == 6) {
            setPathState(
              getPathState(PathType.ALIGN, AlignNum)
            );
            fireNum = 0;
          }
        }
      }
      
    }
    private void doEnd()                                          {
      // Prep For Firing
        IOtake.runIntake(0);
        IOtake.runOtkGate(
          IOTAKE.OuttakeServoState .CLOSED
        );
        IOtake.runFlywheel(
          IOTAKE.FlywheelState .OFF
        );
        setPathState(PathState .END );
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
