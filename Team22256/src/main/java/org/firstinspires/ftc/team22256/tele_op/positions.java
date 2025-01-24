package org.firstinspires.ftc.team22256.tele_op;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class positions extends LinearOpMode {
    private DcMotor rotator;
    private DcMotor slide;
    private Servo S1;
    private Servo S2;
    private Servo wrist;
    private Servo flapper;
    private Servo claw;
    @Override
    public void runOpMode() {
        rotator = hardwareMap.get(DcMotor.class,"rotator");
        slide = hardwareMap.get(DcMotor.class,"slide");
        S1 = hardwareMap.get(Servo.class,"S1");
        S2 = hardwareMap.get(Servo.class,"S2");
        wrist = hardwareMap.get(Servo.class,"wrist");
        flapper = hardwareMap.get(Servo.class,"flapper");
        claw = hardwareMap.get(Servo.class,"claw");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(0);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("S1 position",S1.getPosition());
            telemetry.addData("S2 position",S2.getPosition());
            telemetry.addData("wrist position",wrist.getPosition());
            telemetry.addData("flapper position",flapper.getPosition());
            telemetry.addData("claw position",claw.getPosition());
            telemetry.addData("rotator position",rotator.getCurrentPosition());
            telemetry.update();
        }
    }
}