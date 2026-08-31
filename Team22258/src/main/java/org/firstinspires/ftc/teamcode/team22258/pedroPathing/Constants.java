package org.firstinspires.ftc.teamcode.team22258.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
      .mass(8.355)
      .forwardZeroPowerAcceleration(-41.8651031650941)
      .lateralZeroPowerAcceleration(-78.07340199855834)
      .translationalPIDFCoefficients(new PIDFCoefficients(0.28, 0, 0.03, 0.035))
      .headingPIDFCoefficients(new PIDFCoefficients(1.5,0,0.2,0.035))
    ;
    
    public static MecanumConstants driveConstants = new MecanumConstants()
      .maxPower(1)
      .xVelocity(59.155711256612946)
      .yVelocity(46.97990982175812)
      .leftFrontMotorName("FLMotor")
      .rightFrontMotorName("FRMotor")
      .leftRearMotorName("BLMotor")
      .rightRearMotorName("BRMotor")
      .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
      .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
      .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
      .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
    ;
    
    public static PinpointConstants localizerConstants = new PinpointConstants()
      .forwardPodY(-1.125)
      .strafePodX(-0.75)
      .distanceUnit(DistanceUnit.INCH)
      .hardwareMapName("pinpoint")
      .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
      .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
      .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
    ;
    
    public static PathConstraints pathConstraints = new PathConstraints(
      0.99,
      100,
      .88,
      1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .pinpointLocalizer(localizerConstants)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .build();
    }
}
