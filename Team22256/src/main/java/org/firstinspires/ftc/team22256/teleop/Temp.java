
package org.firstinspires.ftc.team22256.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class Temp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor one;
    private DcMotor two;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        one  = hardwareMap.get(DcMotor.class, "one");
        two  = hardwareMap.get(DcMotor.class, "two");

        two.setDirection(DcMotorSimple.Direction.REVERSE);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        boolean BP = false;

        waitForStart();
        runtime.reset();
        double power = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

           if (gamepad1.a){
               two.setPower(1);
               one.setPower(1);
            } else if(gamepad1.b){
                two.setPower(-1);
                one.setPower(-1);
            } else {
                two.setPower(0);
                one.setPower(0);
            }

            /*
            if (gamepad1.right_bumper && !BP) {
                BP = true;
                power += 0.1;
            } else if (!gamepad1.right_bumper && BP) {
                BP = false;
                power += 0.1;
            }

            if (gamepad1.left_bumper && !BP) {
                BP = true;
                power -= 0.1;
            } else if (!gamepad1.left_bumper && BP) {
                BP = false;
                power -= 0.1;
            }

            if(power < 0 || power >1){
                power = 0;
            }

            two.setPower(power);
            one.setPower(power);
  */
        }
    }
}
