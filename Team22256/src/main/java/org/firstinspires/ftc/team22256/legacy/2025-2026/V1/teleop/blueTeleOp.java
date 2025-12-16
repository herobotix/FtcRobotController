
package org.firstinspires.ftc.team22256.teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp

public class blueTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeft, backRight, frontLeft, frontRight;
    private DcMotorEx shooterLeft, shooterRight;
    private DcMotor intake;
    private DcMotor turret;
    private CRServo kicker;
    private Limelight3A limelight;
    private PIDFController Controller0;
    private double  error= 0;
    private double currentTurretPos = 0;
    private final double TICKS_PER_DEGREE = (double) 116 /180;
    private double turretOutput = 0;
    private static double p = 0.04;
    private static double i  =0;
    private static double d = 0.001;
    private static double f = 0;


    private boolean t_intake = false;

    private enum LauncherState {
        IDLE,
        SHOOTER_SPEED,
        INTAKE_WHEEL_AND_KICKER,

    }

    private LauncherState launcherstate = LauncherState.IDLE;

    @Override
    public void runOpMode() {


        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        turret = hardwareMap.get(DcMotor.class, "turret");

        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        kicker = hardwareMap.get(CRServo.class, "kicker");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Controller0 = new PIDFController(p,i,d,f);

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();

                currentTurretPos = turret.getCurrentPosition();
                error = result.getTx() * TICKS_PER_DEGREE;//Distance from limelight to AprilTag in degrees
                double target = (currentTurretPos + error);
                turretOutput = Controller0.calculate(currentTurretPos, target);//Use PID to calculate output
                turret.setPower(turretOutput);




           /*
            NOT IN USE
            if (gamepad2.right_trigger > 0.2f) {
                kicker.setPower(1);
            } else {
                kicker.setPower(0);
            }

            if (gamepad2.left_trigger > 0.2f){
                shooterLeft.setPower(-0.75 );
                shooterRight.setPower(0.75);

            } else if(gamepad2.left_bumper) {
                shooterLeft.setPower(-0.5);
                shooterRight.setPower(0.5);
            }
            if (gamepad2.right bumper);
            } else {
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
            }
            ARCHIVED
            */

            if (gamepad2.left_bumper) {
                switch (launcherstate) {
                    case IDLE:
                        shooterLeft.setVelocity(-1325);
                        shooterRight.setVelocity(1325);
                        launcherstate = LauncherState.SHOOTER_SPEED;
                        break;
                    case SHOOTER_SPEED:
                        if (shooterLeft.getVelocity() <= -1310 &&
                                shooterRight.getVelocity() > 1330) {
                            launcherstate = LauncherState.INTAKE_WHEEL_AND_KICKER;
                        }
                        break;
                    case INTAKE_WHEEL_AND_KICKER:
                        kicker.setPower(1);
                        intake.setPower(1);
                        break;
                }
            } else if(!gamepad2.left_bumper & !gamepad2.right_bumper){
                launcherstate = LauncherState.IDLE;
                shooterLeft.setVelocity(-1000);
                shooterRight.setVelocity(1000);
                kicker.setPower(0);
                //intake.setPower(0);
            }

            if (gamepad2.right_bumper) {
                switch (launcherstate) {
                    case IDLE:
                        shooterLeft.setVelocity(-1600);
                        shooterRight.setVelocity(1600);
                        launcherstate = LauncherState.SHOOTER_SPEED;
                        break;
                    case SHOOTER_SPEED:
                        if (shooterLeft.getVelocity() <= -1340 &&
                                shooterRight.getVelocity() > 1360) {
                            launcherstate = LauncherState.INTAKE_WHEEL_AND_KICKER;
                        }
                        break;
                    case INTAKE_WHEEL_AND_KICKER:
                        kicker.setPower(1);
                        intake.setPower(1);
                        break;
                }
            } else if(!gamepad2.left_bumper & !gamepad2.right_bumper){
                launcherstate = LauncherState.IDLE;
                shooterLeft.setVelocity(-1000);
                shooterRight.setVelocity(1000);
                kicker.setPower(0);
                //intake.setPower(0);
            }

            t_intake = gamepad2.bWasPressed() == !t_intake;
        if(launcherstate != LauncherState.INTAKE_WHEEL_AND_KICKER) {
            if (gamepad2.a) {
                intake.setPower(-1);   // reverse (Outtake)
            } else if (t_intake) {
                intake.setPower(1);    // forward (Intake)
            } else {
                intake.setPower(0);    // off
            }
        }
           /* if (gamepad2.a) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
        }*/


            /*if (gamepad2.aWasPressed()) {
                if (intake.getPower() == 0) {
                    intake.setPower(1);
                } else  {
                    intake.setPower(0);
                }
            } else if (gamepad2.b) {
                intake.setPower(-1);
            } else if (intake.getPower() == -1) {
                intake.setPower(0);
            }
*/

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            telemetry.addData("left stick x", gamepad1.left_stick_x);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftPower = frontLeftPower / 1.25;
            backLeftPower = backLeftPower  / 1.25;
            frontRightPower = frontRightPower / 1.25;
            backRightPower = backRightPower / 1.25;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            telemetry.addData("y", y);
            telemetry.addData("x", x);
            telemetry.addData("rx", rx);
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("backLeftPower", backLeftPower);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("backRightPower", backRightPower);
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("shooter left velocity",shooterLeft.getVelocity());
            telemetry.addData("shooter right velocity",shooterRight.getVelocity());
            telemetry.update();

        }
    }
}