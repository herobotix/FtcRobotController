package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class  Sorter {
    private Servo kicker1, kicker2, kicker3;
    private ColorSensor sensor1, sensor2, sensor3;

    public enum sorterState {
        KICK,
        SENSE,
    }
    private sorterState sorterState;

    public Sorter(HardwareMap hardwareMap){
        kicker1 = hardwareMap.get(Servo.class,"sorter");
    }



}
