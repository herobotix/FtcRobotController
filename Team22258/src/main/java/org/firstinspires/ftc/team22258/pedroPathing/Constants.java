package org.firstinspires.ftc.team22258.pedroPathing;

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
      .forwardZeroPowerAcceleration(-49.76771140890383)
      .lateralZeroPowerAcceleration(-86.73481051091882)
      .translationalPIDFCoefficients(new PIDFCoefficients(0.28, 0, 0.03, 0.052))
    ;
    
    public static MecanumConstants driveConstants = new MecanumConstants()
      .maxPower(1)
      .xVelocity(56.806623323695874)
      .yVelocity(46.86705053134229)
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
      1,
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
