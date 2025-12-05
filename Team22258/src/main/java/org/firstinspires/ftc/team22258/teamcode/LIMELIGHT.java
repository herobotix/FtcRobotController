package org.firstinspires.ftc.team22258.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;

public class LIMELIGHT {
  
  private Limelight3A limelight;
  
  public void Init() {
    //Initialize Limelight
    limelight.pipelineSwitch(0);
    limelight.start();
  }
  
  public LIMELIGHT() {
  
  }
  
}
