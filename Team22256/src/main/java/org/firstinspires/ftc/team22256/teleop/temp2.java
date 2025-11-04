
package org.firstinspires.ftc.team22256.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp

public class temp2 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor testMotor;

    @Override
    public void runOpMode() {

        testMotor = hardwareMap.get(DcMotor.class, "testMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                testMotor.setPower(1);
            } else if(gamepad1.b){
                testMotor.setPower(-1);
            } else {
                testMotor.setPower(0);
            }

        }
    }
}
