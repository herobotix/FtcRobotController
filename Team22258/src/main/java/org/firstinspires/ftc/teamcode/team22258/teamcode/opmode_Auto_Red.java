package org.firstinspires.ftc.teamcode.team22258.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.AUTO;

@Autonomous(name = "Opmode (Auto, Red) [1.2.22]",  group = "Autonomous")
@Configurable
public class opmode_Auto_Red  extends LinearOpMode {
  //Autonomous Opmode on Red
  
  // Run Function
    @Override
    public void runOpMode() {
      //Begin
      
      // Class Definitions
      AUTO auto = new AUTO(telemetry, true);
      
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