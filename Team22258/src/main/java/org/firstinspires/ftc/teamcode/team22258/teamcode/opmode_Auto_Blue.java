package org.firstinspires.ftc.team22258.teamcode;

import static org.firstinspires.ftc.teamcode.team22258.teamcode.classes.AUTO.Version;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.AUTO;

/**Autonomous Opmode on Blue*/
@Autonomous(name = "Opmode (Auto, Blue) ["+ Version +"]", group = "Autonomous")
public class opmode_Auto_Blue extends AUTO {
  public opmode_Auto_Blue() {
    isRedAlliance = false;
  }
}