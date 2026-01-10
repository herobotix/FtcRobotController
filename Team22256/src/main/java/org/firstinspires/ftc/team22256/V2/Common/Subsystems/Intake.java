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

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;


public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();
    private Intake(){ }
    private MotorEx intake = new MotorEx("intake");

    private  double intakePower = 1;
    private  double outtakePower = -1;
    private final int stopPower = 0;

    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }
    public void setOuttakePower(double outtakePower){
        this.outtakePower = outtakePower;
    }

    public Command intaking = new LambdaCommand()
            .setStart(() -> intake.setPower(intakePower))
            .setIsDone(() -> true)
            .requires(this);
    public Command outtaking = new LambdaCommand()
            .setStart(() -> intake.setPower(outtakePower))
            .setIsDone(() -> true);

    public Command stop = new LambdaCommand()
            .setStart(() -> intake.setPower(stopPower))
            .setIsDone(() -> true);
    public double getIntakePower() {
        return intakePower;
    }

    public double getOuttakePower() {
        return outtakePower;
    }





    }






