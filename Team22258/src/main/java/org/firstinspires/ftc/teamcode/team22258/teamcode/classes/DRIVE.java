package org.firstinspires.ftc.teamcode.team22258.teamcode.classes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Configurable
public class DRIVE {

  private DcMotor FLMotor, FRMotor, BLMotor, BRMotor;
  private IMU rIMU;

    double
            FLMP,
            FRMP,
            BLMP,
            BRMP;

  // Driving Definitions
    boolean fieldCentric
            = false;
    double
            MPN,
            powHead,
            powSide,
            powTurn;

  // Position Definitions
    double robotYawRadians;
    double targetRot;

    public DRIVE(DcMotor FLMotor, DcMotor FRMotor, DcMotor BLMotor, DcMotor BRMotor, IMU rIMU){
            this.FLMotor = FLMotor;
            this.FRMotor = FRMotor;
            this.BLMotor = BLMotor;
            this.BRMotor = BRMotor;
            this.rIMU = rIMU;
    }

    public void Init(HardwareMap hardwareMap){
        FLMotor        = hardwareMap.get( DcMotor .class, "FLMotor"       );
        FRMotor        = hardwareMap.get( DcMotor .class, "FRMotor"       );
        BLMotor        = hardwareMap.get( DcMotor .class, "BLMotor"       );
        BRMotor        = hardwareMap.get( DcMotor .class, "BRMotor"       );

        IMU.Parameters parameters =
                new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection .UP,
                        RevHubOrientationOnRobot.UsbFacingDirection  .BACKWARD
                )
                );
        rIMU.initialize(parameters);

      // Set Robot Rotation Value
        robotYawRadians = rIMU.getRobotYawPitchRollAngles().getYaw( AngleUnit.RADIANS );
        targetRot = robotYawRadians;

      // Set Motor Behaviors
        FLMotor .setDirection( DcMotor.Direction .REVERSE );
        FRMotor .setDirection( DcMotor.Direction .FORWARD );
        BLMotor .setDirection( DcMotor.Direction .FORWARD );
        BRMotor .setDirection( DcMotor.Direction .FORWARD );

    }

    public void input(Gamepad gamepad1){
        if (gamepad1.start) {rIMU.resetYaw();}

      // Inputs
        fieldCentric = gamepad1.bWasPressed() == (!fieldCentric); // Field-Centrism Toggle
        powHead = -gamepad1.left_stick_y;
        powSide = gamepad1.left_stick_x;
        powTurn = (
                /*(Limelight.getTargetLock() != LIMELIGHT.TargetLock .OFF )? (Limelight.getPowTurn()):*/
                (gamepad1.right_stick_x)
        );
    }

    public void move(){
      //Movement Code

      // Get Robot Rotation Value
        robotYawRadians = rIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit .RADIANS );

      // Field Centric
        if (fieldCentric) {
            powHead = powSide * Math.sin(robotYawRadians) + powHead * Math.cos(robotYawRadians);
            powSide = powSide * Math.cos(robotYawRadians) + powHead * Math.sin(robotYawRadians);
        }

      // Drive & Strafe & Rotate
        FLMP = powHead + powSide + powTurn;
        FRMP = powHead - powSide - powTurn;
        BLMP = powHead - powSide + powTurn;
        BRMP = powHead + powSide - powTurn;

      // Power Control
        MPN = (
                Math.max(
                        Math.max(
                                Math.max( Math.abs(FLMP), Math.abs(FRMP) ),
                                Math.max( Math.abs(BLMP), Math.abs(BRMP) )
                        ),
                        (1)
                )
        );
        if (MPN > 1){
            BRMP /= MPN;
            BLMP /= MPN;
            FRMP /= MPN;
            FLMP /= MPN;
        }

      // Trigger Motors
        FLMotor.setPower(FLMP);
        FRMotor.setPower(FRMP);
        BLMotor.setPower(BLMP);
        BRMotor.setPower(BRMP);

    }

    public void zeroMotors(){
    FRMP = 0;
    FLMP = 0;
    BRMP = 0;
    BLMP = 0;
    }

}