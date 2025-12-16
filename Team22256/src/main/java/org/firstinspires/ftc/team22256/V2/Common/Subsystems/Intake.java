package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Intake {

    private DcMotor intake;

    public enum IntakeState{
        INTAKING,
        OUTAKING,
        STOPPED;
    }

    private IntakeState intakeState;
    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class,"intake");
    }

    public IntakeState getIntakeState(){
        return intakeState;
    }
    public void setIntakeState(IntakeState intakeState){
        this.intakeState = intakeState;
    }
    public void updateIntake(){
        switch (intakeState){
            case INTAKING:
                intake.setPower(1);
                break;
            case OUTAKING:
                intake.setPower(-1);
                break;
            case STOPPED:
                intake.setPower(0);
                break;
            default:
                intakeState = IntakeState.STOPPED;
        }
    }
}
