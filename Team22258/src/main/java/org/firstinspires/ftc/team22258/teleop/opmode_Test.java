package org.firstinspires.ftc.team22258.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@Config
@TeleOp(name = "Opmode (Test)")
public class opmode_Test extends LinearOpMode {

  private DcMotor Arm;

  
  @Override
  public void runOpMode() {
      Arm = hardwareMap.get(DcMotor.class, "Arm");
      Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      Arm.setTargetPosition(3550);
      Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Arm.setPower(0.003);
      
      waitForStart();
      while (opModeIsActive()) {
          telemetry.addData("ArmPwr", Arm.getPower());
          telemetry.addData("ArmPos", Arm.getCurrentPosition());
          telemetry.update();
      }
    
  }
  
}