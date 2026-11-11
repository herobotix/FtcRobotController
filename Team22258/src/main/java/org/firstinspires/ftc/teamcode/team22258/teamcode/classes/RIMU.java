package org.firstinspires.ftc.teamcode.team22258.teamcode.classes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class RIMU {
  // IMU & Position Code
  
  // Definitions
    double robotYawRadians;
    private IMU rIMU;
    
  // Functions
    public void   Init          (HardwareMap      hardwareMap    )  {
      // Initialization Code
      
      // Map Hardware
        rIMU = hardwareMap.get( IMU .class,"rIMU");
  
      // Set Robot Orientation (IMU)
        IMU.Parameters parameters =
          new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection .UP,
            RevHubOrientationOnRobot.UsbFacingDirection  .BACKWARD
          )
        );
        rIMU.initialize(parameters);
        
      // Set Robot Rotation Value
        robotYawRadians = rIMU.getRobotYawPitchRollAngles().getYaw( AngleUnit .RADIANS );
        
    }
    public void   doTelemetry   (TelemetryManager panelsTelemetry)  {
      // Preform Telemetry for IMU Code
        panelsTelemetry.debug("Current Robot Yaw",  robotYawRadians / (2*Math.PI) );
        
    }
    public double getYawRadians (                                )  {
      // Get Robot Rotation Value
        return robotYawRadians = rIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit .RADIANS );
    }
    public void   resetYaw      (                                )  {
      // Reset Robot Yaw Value
        rIMU.resetYaw();
    }
    
}