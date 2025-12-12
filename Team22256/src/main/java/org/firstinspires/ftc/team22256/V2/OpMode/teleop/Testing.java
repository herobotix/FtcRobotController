
package org.firstinspires.ftc.team22256.V2.OpMode.teleop;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Configurable
@TeleOp
public class Testing extends LinearOpMode {

    private DcMotor frontLeft,frontRight,backLeft,backRight;



    @Override
    public void runOpMode() throws InterruptedException
    {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {


        if(gamepad1.a){
            frontLeft.setPower(1);
        }else if(gamepad1.b){
            frontRight.setPower(1);
        } else if(gamepad1.x){
            backRight.setPower(1);
        } else if(gamepad1.y){
            backLeft.setPower(1);
        } else{
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }

//fl back
            //fr forward
            //bl forward
            //br back




        }

    }
}
