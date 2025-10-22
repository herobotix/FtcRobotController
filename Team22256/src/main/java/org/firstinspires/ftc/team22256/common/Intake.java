package org.firstinspires.ftc.team22256.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private DcMotor intakeMotor;
    Intake(){
        intakeMotor  = hardwareMap.get(DcMotor.class, "intakeMotor");
    }
    public void intakeGameElement(double power){
        intakeMotor.setPower(power);
    }










}
