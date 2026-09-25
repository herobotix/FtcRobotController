package org.firstinspires.ftc.team22258.teamcode;

import static org.firstinspires.ftc.team22258.teamcode.opmode_Auto_SIMPLE.Version;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team22258.pedroPathing.Constants;

@Autonomous(name = "Opmode (Auto, SIMPLE) ["+ Version +"]", group = "Autonomous")
@Configurable
public class opmode_Auto_SIMPLE extends LinearOpMode {
  //Simplified Autonomous Code
  
  // Version Number Definition
  public static final String Version
    = "1.2.9";
  
  // Path Definitions
    public static class Paths {
      
      public PathChain MOVE;
      
      public Paths(Follower follower) {
        MOVE = follower.pathBuilder       ()
          .addPath
            (new BezierLine(
              new Pose(
                72.000,
                8.500
              ),
              new Pose(
                  72.000,
                  30.000
              )
            ))
          .setTangentHeadingInterpolation ()
          .build                          ();
      }
    }
    private Paths paths;
  
  // Follower Definition
    public Follower follower;
  
  // Telemetry Definition
    private TelemetryManager panelsTelemetry;
    
  // Run Function
    @Override
    public void runOpMode     ()      {
      //Begin
      
      // Init & Wait
        doInit();
        waitForStart ();
        
      // Run Opmode
        if    ( opModeIsActive () ) doRun  ();
        while ( opModeIsActive () ) doLoop ();
        
    }
    
  // Primary Functions
    private void doInit       ()      {
      // Initialization Code
      
      // init paths
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
        
        paths = new Paths(follower); // Build paths
        
      // Telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        
    }
    private void doRun        ()      {
      //Run Code
      
      
      follower.followPath(paths.MOVE, true);
      
    }
    private void doLoop       ()      {
      //Loop Code
      
      // Update Pedro Pathing
      follower.update();
      
      // Telemetry
      doTelemetry();
      
    }
    
  // Secondary Functions
    private void doTelemetry  ()      {
      //Telemetry Code
      
      // Log values to Panels and Driver Station
      panelsTelemetry.debug("X", follower.getPose().getX());
      panelsTelemetry.debug("Y", follower.getPose().getY());
      panelsTelemetry.debug("Heading", follower.getPose().getHeading());
      panelsTelemetry.update(telemetry);
    }
    
}