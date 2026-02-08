
package org.firstinspires.ftc.team22256.V2.OpMode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Turret;


@Autonomous

public class MoveAuto extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor backLeft,backRight,frontLeft,frontRight;











    @Override
    public void runOpMode() {


        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {
        runtime.reset();

            while(runtime.seconds() < 0.5){
                backLeft.setPower(1);
                backRight.setPower(-1);
                frontRight.setPower(1);
                frontLeft.setPower(-1);
            }

            telemetry.update();
            break;
        }
    }
}
