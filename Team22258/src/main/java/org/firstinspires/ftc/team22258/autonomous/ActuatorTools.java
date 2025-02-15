/*package org.firstinspires.ftc.team22258.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ActuatorTools {
    private DcMotor Arm;
    private Servo LClaw, RClaw;
    private IMU rIMU;
    
    private String LAMotorName = "wrist";
    private String ClawServoName = "claw";
    private String IntakeServoName = "intake";

    // Arm Positions
    private static int armMin = 0;
    private static int armMax = 4650;

    // Claw predefined positions
    private static double clawLO = 0.25;
    private static double clawLC = 0.75;
    private static double clawRO = 0.25;
    private static double clawRC = 0.75;
    
    private int targetUpperArm, targetLowerArm;
    private static double lastUAPosActual, totalUAPosInt=0;

    // Initialize Class
    public ActuatorTools(HardwareMap hardwareMap) {
        // Map the hardware actuator variables.
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        LClaw = hardwareMap.get(Servo.class, "LClaw");
        RClaw = hardwareMap.get(Servo.class, "RClaw");
        rIMU = hardwareMap.get(IMU.class, "rIMU");
        rIMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));
    }

    public class LowBasketHover implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet, double iAngle) {
            int ArmPos = Arm.getCurrentPosition();
            int ArmTarget = (int)(iAngle / RpT);

            if (!initialized) {
                Arm.setTargetPosition(ArmTarget);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(1);
                lastUAPosActual = ArmPos;
                initialized = true;
            }

            double UAAng = Angle(ArmPos,UA_rPt,UA_pOt);
            UAMotor.setPower( Power(ArmPos, ArmTarget, 0, UA_kP, UA_kI, UA_kD, UA_kA, UAAng) );
            packet.put("UAMotorPos", ArmPos);
            return Math.abs(ArmPos - ArmTarget)<1;
        }
    }

    // UpperArm PIDF Coefficients
    private static double UA_kA=0;      // Must be less than 1.
    private static double UA_kP=0;
    private static double UA_kI=0;
    private static double UA_kD=0;
    private static double RpT = 2*Math.PI/(28.0*2.89*3.61*5.23*(90.0/45.0)*(125.0/45.0));  //Radians per Tick
    private static double UA_pOt = 600;                                                       //Offset Position in Ticks

    public double Angle(int Pos, double Slope, double Offset) { return (Slope * (Pos + Offset)); }
    public double Power(int Pos, int Tgt, double kF, double kP, double kI, double kD, double kA, double Ang) {
        double Dif = Pos - Tgt;
        double Der = Pos - lastUAPosActual;
        double Int = Dif + totalUAPosInt;
        lastUAPosActual = Pos;
        totalUAPosInt = Int;
        return ((1-kA)*((Math.signum(Dif)*kF) + (-Dif*kP) + (Int*kI) - (Der*kD)) + Math.sin(Ang)*kA);
    }

}

*/