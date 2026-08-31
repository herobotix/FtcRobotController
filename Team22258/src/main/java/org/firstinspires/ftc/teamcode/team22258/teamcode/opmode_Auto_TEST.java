package org.firstinspires.ftc.teamcode.team22258.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team22258.pedroPathing.Constants;


@Autonomous(name = "Opmode (Auto, TEST) [----]", group = "Autonomous")
@Configurable
public class opmode_Auto_TEST extends LinearOpMode {
  
  public static class Paths {
    
    public PathChain SETUP;
    
    public Paths(Follower follower) {
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
  
  @Override
  public void runOpMode() {
    //Begin
    
    // Init & Wait
      Fn_Init();
      waitForStart();
      
    // Run Opmode
      if (opModeIsActive()) {
        follower.update(); // Update Pedro Pathing
        follower.followPath(paths.SETUP, true);
      }
      while (opModeIsActive()) {
      follower.update(); // Update Pedro Pathing
      
      // Log values to Panels and Driver Station
      panelsTelemetry.debug("X", follower.getPose().getX());
      panelsTelemetry.debug("Y", follower.getPose().getY());
      panelsTelemetry.debug("Heading", follower.getPose().getHeading());
      panelsTelemetry.update(telemetry);
    }
    
  }
  
  private void Fn_Init() {
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
  
}