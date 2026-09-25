package org.firstinspires.ftc.teamcode.team22258.teamcode;

import static org.firstinspires.ftc.teamcode.team22258.teamcode.classes.AUTO.Version;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.team22258.teamcode.classes.AUTO;

/**Autonomous Opmode on Red */
@Autonomous(name = "Opmode (Auto, Red)  ["+ Version +"]", group = "Autonomous")
public class opmode_Auto_Red  extends AUTO {
  public opmode_Auto_Red()  {
    isRedAlliance = true ;
  }
}