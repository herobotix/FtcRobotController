
package org.firstinspires.ftc.team22256.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        while (opModeIsActive()) {
            switch (State) {
                case DRIVE:
                    robot.DriveToPosition(63.5,1000);
                    robot.strafeToPosition(24,500);
                    robot.turn(45,250);
                    break;
                case SHOOT:

                    break;
            }

            telemetry.update();
        }
    }
}
