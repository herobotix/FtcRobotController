package org.firstinspires.ftc.team22256.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    private DcMotor frontRight,backRight,frontLeft,backLeft;
    Drivetrain drivetrain;
    Intake intake;
    Turret turret;
    public Robot(HardwareMap hardwareMap){
        drivetrain = new Drivetrain(hardwareMap);
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        intake = new Intake();
        turret = new Turret();

    }





}
