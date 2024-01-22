package org.firstinspires.ftc.teamcode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Park&PlacePixel-Blue", group="autonomous")
//@Disabled
public class blueParkAndPixel extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor hdhex = null;
    //private DcMotor leftBackDrive = null;

    public void runOpMode() {
        hdhex = hardwareMap.get(DcMotor.class, "hdhex");
        hdhex.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            double hdHexPower = 1.0;

            while (opModeIsActive()) {
                hdhex.setPower(hdHexPower);
            }
        }
    }


}
