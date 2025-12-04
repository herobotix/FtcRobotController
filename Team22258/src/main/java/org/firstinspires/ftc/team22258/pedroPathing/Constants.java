package org.firstinspires.ftc.team22258.pedroPathing;

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
    ;
    
    public static MecanumConstants driveConstants = new MecanumConstants()
      .maxPower(1)
      .leftFrontMotorName("FLMotor")
      .rightFrontMotorName("FRMotor")
      .leftRearMotorName("BLMotor")
      .rightRearMotorName("BRMotor")
      .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
      .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
      .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
      .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
    ;
    
    public static PinpointConstants localizerConstants = new PinpointConstants()
      .forwardPodY(-3)
      .strafePodX(-4.125)
      .distanceUnit(DistanceUnit.INCH)
      .hardwareMapName("pinpoint")
      .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
      .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
      .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
    ;
    
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .pinpointLocalizer(localizerConstants)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .build();
    }
}
