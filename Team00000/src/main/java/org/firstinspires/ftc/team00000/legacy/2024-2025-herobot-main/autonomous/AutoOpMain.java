package org.firstinspires.ftc.team00000.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="AutoOpMain", group="autonomous")
//@Disabled
public class AutoOpMain extends LinearOpMode {
    private DcMotorEx BLMotor;
    private DcMotorEx BRMotor;
    private DcMotorEx FLMotor;
    private DcMotorEx FRMotor;
    private IMU imu;

    //initializing stick axis and power level variables
    private double LSx, rLSx, LSy, rLSy, RSx;
    private double FRPwr, BRPwr, FLPwr, BLPwr, MaxPwr;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables.
        FLMotor  = hardwareMap.get(DcMotorEx.class, "FLMotor");
        BLMotor = hardwareMap.get(DcMotorEx.class, "BLMotor");
        FRMotor  = hardwareMap.get(DcMotorEx.class, "FRMotor");
        BRMotor = hardwareMap.get(DcMotorEx.class, "BRMotor");
        imu = hardwareMap.get(IMU.class,"imu");

        // Stop and reset encoders
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to use encoders
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        ));

        ElapsedTime timer = new ElapsedTime();
        LSx=0.0;
        LSy=0.0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        timer.reset();
        // Run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            if (timer.time() < 1.0) {
                //if (gamepad1.start) { imu.resetYaw(); }
                //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                LSx = -Math.sin(Math.toRadians(30));
                LSy = Math.cos(Math.toRadians(30));
                rLSx = LSx; //LSx*Math.cos(-botHeading)-LSy*Math.sin(-botHeading);
                rLSy = LSy; //LSx*Math.sin(-botHeading)+LSy*Math.cos(-botHeading);
                RSx = 0.0;

                telemetry.addData("LSy", LSy);
                telemetry.addData("LSx", LSx);
                telemetry.addData("RSx", RSx);

                MaxPwr = Math.max(Math.abs(rLSx) + Math.abs(rLSy) + Math.abs(RSx), 1);
                // Change MaxPwr to allow higher average power ... find max of FLPwr, BLPwr, FRPwr, BRPwr, 1
                FLPwr = (rLSy + rLSx + RSx) / MaxPwr;
                BLPwr = (rLSy - rLSx + RSx) / MaxPwr;
                FRPwr = (rLSy - rLSx - RSx) / MaxPwr;
                BRPwr = (rLSy + rLSx - RSx) / MaxPwr;

                // Handle state transitions based on gamepad input
                FLMotor.setPower(FLPwr);
                BLMotor.setPower(BLPwr);
                FRMotor.setPower(FRPwr);
                BRMotor.setPower(BRPwr);

                telemetry.update();
            }
        }
       
       
  
    }

}