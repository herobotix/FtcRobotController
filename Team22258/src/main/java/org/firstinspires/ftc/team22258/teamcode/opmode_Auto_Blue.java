package org.firstinspires.ftc.team22258.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team22258.teamcode.classes.AUTO;

@Autonomous(name = "Opmode (Auto, Blue) [1.2.21]", group = "Autonomous")
@Configurable
public class opmode_Auto_Blue extends LinearOpMode {
  //Autonomous Opmode on Blue
  
  // Run Function
    @Override
    public void runOpMode() {
      //Begin
      
      // Class Definitions
        AUTO auto = new AUTO(telemetry, false);
        
      // Init & Wait
        auto.Init(hardwareMap);
        waitForStart();
        
      // Run Opmode
        while (opModeIsActive()) {
          auto.doLoop();
          if (gamepad1.a) {
            while (opModeIsActive()) {}
          }
        }
        
    }
    
}