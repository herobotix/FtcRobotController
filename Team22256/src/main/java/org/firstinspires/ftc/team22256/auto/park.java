package org.firstinspires.ftc.team22256.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous


public class park extends LinearOpMode {

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rotator;
    private DcMotor slide;
    private Servo S1;
    private Servo S2;
    private Servo wrist;
    private Servo flapper;
    private Servo claw;
    private PIDController controller0;
    public static double p=0,i=0,d=0;
    public static double f = 0;
    double pid = 0;
    double ff = 0;
    double power = 0;
    public static double target = 0;
    public static double ticks_in_degree = 4.687;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        rightFront  = hardwareMap.get(DcMotor.class,"rightFront");
        leftFront  = hardwareMap.get(DcMotor.class,"leftFront");
        rightBack = hardwareMap.get(DcMotor.class,"rightBack");
        leftBack = hardwareMap.get(DcMotor.class,"leftBack");
        rotator = hardwareMap.get(DcMotor.class,"rotator");
        slide = hardwareMap.get(DcMotor.class,"slide");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller0  = new PIDController(p,i,d); // Initialization of controller and coeffients

        pid = 0;
        ff = 0;
        power = 0;

        double changePower = 1.25;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            controller0.setPID(p,i,d);
            int slidePos = slide.getCurrentPosition();
            pid = controller0.calculate(slidePos,target);
            ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            power = pid + ff;
            slide.setPower(power);















            telemetry.update();
        }
    }
}