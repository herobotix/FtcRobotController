package org.firstinspires.ftc.team22258.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name = "Net", group="autonomous")
public class NetAuto extends LinearOpMode {

    private Servo Wrist;
    private Servo LClaw;
    private Servo RClaw;
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor Arm;

    int FLMotorPower;
    int ClawState;
    int ArmPower;
    int FRMotorPower;
    int WristState;
    int BLMotorPower;
    int Wriston;
    int Clawn;
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
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        LClaw = hardwareMap.get(Servo.class, "LClaw");
        RClaw = hardwareMap.get(Servo.class, "RClaw");
        FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
        FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
        BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
        BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");
        Arm = hardwareMap.get(DcMotor.class, "Arm");

        Prepare();
        waitForStart();
        if (opModeIsActive()) {
            if (true) {
                if (true) {
                    Arm2(0.35);
                    Continue(0.4, 1);
                }
                END();
                if (true) {
                    Head(0.05);
                    Side(0);
                    Turn(0);
                    Arm2(0);
                    Wristoggle(0);
                    Clawggle(0);
                    Continue(0.25, 1);
                }
                END();
                if (true) {
                    Head(-1);
                    Continue(0.2, 1);
                }
                END();
                if (true) {
                    Side(1);
                    Continue(0.9, 1);
                }
                END();
                if (true) {
                    Head(-0.05);
                    Continue(0.04, 1);
                }
                END();
                if (true) {
                    Arm2(0.5);
                    Continue(0.67, 1);
                }
            }
            END();
            while (opModeIsActive()) {
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Telemetry2() {
        double iArm = 0.0;

        telemetry.addData("ClawState", ClawState);
        telemetry.addData("WristState", WristState);
        telemetry.addData("Clawn", Clawn);
        telemetry.addData("Wriston", Wriston);
        telemetry.addData("ArmInput", iArm);
        telemetry.addData("ArmPower", ArmPower);
        telemetry.addData("WristPos", Wrist.getPosition());
        telemetry.addData("LClawPos", LClaw.getPosition());
        telemetry.addData("RClawPos", RClaw.getPosition());
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void Move(double iHead, int iSide, int iTurn) {
        Head(iHead);
        Side(iSide);
        Turn(iTurn);
    }

    /**
     * Describe this function...
     */
    private void Clawrm(double iArm, int iWrist_b, int iClaw_b) {
        Arm2(iArm);
        Wristoggle(iWrist_b);
        Clawggle(iClaw_b);
    }

    /**
     * Describe this function...
     */
    private void Arm2(double iArm) {
        if (iArm != 0) {
            ArmPower += iArm;
            // Move Arm
        }
    }

    /**
     * Describe this function...
     */
    private void Head(double iHead) {
        FLMotorPower += iHead * -1;
        FRMotorPower += iHead * -1;
        BLMotorPower += iHead * -1;
        BRMotorPower += iHead * -1;
    }

    /**
     * Describe this function...
     */
    private void Wristoggle(int iWrist_b) {
        if (WristState == 0 && iWrist_b > 0) {
            // claw closed
            WristState = 1;
            // claw open
        } else if (WristState == 1 && iWrist_b == 0) {
            WristState = 2;
            // claw open
        } else if (WristState == 2 && iWrist_b > 0) {
            // claw open
            WristState = 3;
            // claw closed
        } else if (WristState == 3 && iWrist_b == 0) {
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
    private void Clawggle(int iClaw_b) {
        if (ClawState == 0 && iClaw_b > 0) {
            // claw closed
            ClawState = 1;
            // claw open
        } else if (ClawState == 1 && iClaw_b == 0) {
            ClawState = 2;
            // claw open
        } else if (ClawState == 2 && iClaw_b > 0) {
            // claw open
            ClawState = 3;
            // claw closed
        } else if (ClawState == 3 && iClaw_b == 0) {
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
    private void Side(int iSide) {
        FLMotorPower += iSide * -1;
        FRMotorPower += iSide * 1;
        BLMotorPower += iSide * 1;
        BRMotorPower += iSide * -1;
    }

    /**
     * Describe this function...
     */
    private void Turn(int iTurn) {
        FLMotorPower += iTurn * 1;
        FRMotorPower += iTurn * -1;
        BLMotorPower += iTurn * 1;
        BRMotorPower += iTurn * -1;
    }

    /**
     * Describe this function...
     */
    private void Prepare() {
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ClawState = 0;
        WristState = 0;
        Clawn = 0;
        Wriston = 0;
    }

    /**
     * Describe this function...
     */
    private void END() {
        Head(0);
        Side(0);
        Turn(0);
        Arm2(0);
        Wristoggle(0);
        Clawggle(0);
        Continue(1, 0);
    }

    /**
     * Describe this function...
     */
    private void Continue(double Time, int iMove) {
        Telemetry2();
        MotorPowerControl(iMove);
        Update();
        sleep((long) (Time * 1000));
    }

    /**
     * Describe this function...
     */
    private void MotorPowerControl(int iMove) {
        double MotorPowerNormalizer;

        MotorPowerNormalizer = iMove / ((Double) JavaUtil.inListGet(JavaUtil.sort(JavaUtil.createListWith(Math.abs(FLMotorPower), Math.abs(FRMotorPower), Math.abs(BLMotorPower), Math.abs(BRMotorPower)), JavaUtil.SortType.NUMERIC, JavaUtil.SortDirection.DESCENDING), JavaUtil.AtMode.FIRST, (int) 0, false)).doubleValue();
        FLMotorPower = (int) (FLMotorPower * MotorPowerNormalizer);
        FRMotorPower = (int) (FRMotorPower * MotorPowerNormalizer);
        BLMotorPower = (int) (BLMotorPower * MotorPowerNormalizer);
        BRMotorPower = (int) (BRMotorPower * MotorPowerNormalizer);
    }
}