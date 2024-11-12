package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "Manual Drive", group="teleop")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    private DcMotor Arm;
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private Servo LClaw;
    private Servo RClaw;
    private Servo Wrist;

    float ArmInput;
    int ClawState;
    int WristState;
    int FLMotorPower;
    int ArmPower;
    int Clawn;
    int Wriston;
    int FRMotorPower;
    int BLMotorPower;
    int BRMotorPower;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        LClaw = hardwareMap.get(Servo.class, "LClaw");
        RClaw = hardwareMap.get(Servo.class, "RClaw");
        Wrist = hardwareMap.get(Servo.class, "Wrist");

        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ClawState = 0;
        WristState = 0;
        Clawn = 0;
        Wriston = 0;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Move();
                Arm2();
                Wristoggle();
                Clawggle();
                Telemetry2();
                Update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Arm2() {
        ArmInput = gamepad1.left_trigger - gamepad1.right_trigger;
        if (ArmInput != 0) {
            ArmPower += ArmInput * 0.5;
            // Move Arm
        }
    }

    /**
     * Describe this function...
     */
    private void Clawggle() {
        if (ClawState == 0 && gamepad1.left_bumper) {
            // claw closed
            ClawState = 1;
            // claw open
        } else if (ClawState == 1 && !gamepad1.left_bumper) {
            ClawState = 2;
            // claw open
        } else if (ClawState == 2 && gamepad1.left_bumper) {
            // claw open
            ClawState = 3;
            // claw closed
        } else if (ClawState == 3 && !gamepad1.left_bumper) {
            ClawState = 0;
            // claw closed
        }
        if (ClawState == 0 || ClawState == 3) {
            Clawn = 0;
        } else {
            Clawn = 1;
        }
    }

    /**
     * Describe this function...
     */
    private void Wristoggle() {
        if (WristState == 0 && gamepad1.right_bumper) {
            // claw closed
            WristState = 1;
            // claw open
        } else if (WristState == 1 && !gamepad1.right_bumper) {
            WristState = 2;
            // claw open
        } else if (WristState == 2 && gamepad1.right_bumper) {
            // claw open
            WristState = 3;
            // claw closed
        } else if (WristState == 3 && !gamepad1.right_bumper) {
            WristState = 0;
            // claw closed
        }
        if (WristState == 0 || WristState == 3) {
            Wriston = 0;
        } else {
            Wriston = 1;
        }
    }

    /**
     * Describe this function...
     */
    private void Move() {
        double MotorPowerNormalizer;

        if (true) {
            // Drive
            FLMotorPower += gamepad1.left_stick_y * 1;
            FRMotorPower += gamepad1.left_stick_y * 1;
            BLMotorPower += gamepad1.left_stick_y * 1;
            BRMotorPower += gamepad1.left_stick_y * 1;
        }
        if (true) {
            // Strafe
            FLMotorPower += gamepad1.left_stick_x * 1;
            FRMotorPower += gamepad1.left_stick_x * -1;
            BLMotorPower += gamepad1.left_stick_x * -1;
            BRMotorPower += gamepad1.left_stick_x * 1;
        }
        if (true) {
            // Rotate
            FLMotorPower += gamepad1.right_stick_x * -1;
            FRMotorPower += gamepad1.right_stick_x * 1;
            BLMotorPower += gamepad1.right_stick_x * -1;
            BRMotorPower += gamepad1.right_stick_x * 1;
        }
        if (true) {
            // Power Control
            MotorPowerNormalizer = ((Double) JavaUtil.inListGet(JavaUtil.sort(JavaUtil.createListWith(Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_y), Math.abs(gamepad1.right_stick_x)), JavaUtil.SortType.NUMERIC, JavaUtil.SortDirection.DESCENDING), JavaUtil.AtMode.FIRST, (int) 0, false)).doubleValue() / ((Double) JavaUtil.inListGet(JavaUtil.sort(JavaUtil.createListWith(Math.abs(FLMotorPower), Math.abs(FRMotorPower), Math.abs(BLMotorPower), Math.abs(BRMotorPower)), JavaUtil.SortType.NUMERIC, JavaUtil.SortDirection.DESCENDING), JavaUtil.AtMode.FIRST, (int) 0, false)).doubleValue();
            FLMotorPower = (int) (FLMotorPower * MotorPowerNormalizer);
            FRMotorPower = (int) (FRMotorPower * MotorPowerNormalizer);
            BLMotorPower = (int) (BLMotorPower * MotorPowerNormalizer);
            BRMotorPower = (int) (BRMotorPower * MotorPowerNormalizer);
        }
    }

    /**
     * Describe this function...
     */
    private void Update() {
        FLMotor.setPower(FLMotorPower);
        FRMotor.setPower(FRMotorPower * -1);
        BLMotor.setPower(BLMotorPower * -1);
        BRMotor.setPower(BRMotorPower * -1);
        Arm.setPower(ArmPower);
        LClaw.setPosition(1 - Clawn);
        RClaw.setPosition(Clawn);
        Wrist.setPosition(Wriston);
        FLMotorPower = 0;
        FRMotorPower = 0;
        BLMotorPower = 0;
        BRMotorPower = 0;
        ArmPower = 0;
    }

    /**
     * Describe this function...
     */
    private void Telemetry2() {
        telemetry.addData("ClawState", ClawState);
        telemetry.addData("WristState", WristState);
        telemetry.addData("Clawn", Clawn);
        telemetry.addData("Wriston", Wriston);
        telemetry.addData("ArmInput", ArmInput);
        telemetry.addData("ArmPower", ArmPower);
        telemetry.addData("WristPos", Wrist.getPosition());
        telemetry.addData("LClawPos", LClaw.getPosition());
        telemetry.addData("RClawPos", RClaw.getPosition());
        telemetry.update();
    }
}
