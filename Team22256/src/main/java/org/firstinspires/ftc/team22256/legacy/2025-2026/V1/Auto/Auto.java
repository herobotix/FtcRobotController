
package org.firstinspires.ftc.team22256.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Turret;
import org.firstinspires.ftc.team22256.common.Methods;


@Autonomous

public class Auto extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor backLeft,backRight,frontLeft,frontRight;
    private DcMotorEx shooterLeft, shooterRight;
    private DcMotor intake;
    private DcMotor turret;

    private Servo kicker;
    private boolean toggle = false;
    Methods robot = new Methods(hardwareMap);

    public enum state{
        DRIVE,
        SHOOT
    }
    state State = state.DRIVE;









    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        Turret.INSTANCE.setTurretState(Turret.State.IDLE);
        while (opModeIsActive()) {
            Turret.INSTANCE.setTurretState(Turret.State.VISION);
            while(runtime.seconds() < 1.1){
                backLeft.setPower(1);
                backRight.setPower(1);
                frontLeft.setPower(1);
                frontRight.setPower(1);
            }


            telemetry.update();
        }
    }
}
