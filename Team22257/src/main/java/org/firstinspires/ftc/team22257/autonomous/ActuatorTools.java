package org.firstinspires.ftc.team22257.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ActuatorTools {
    private DcMotor UAMotor, LAMotor;
    private Servo ClawServo;
    private CRServo IntakeServo;
    private IMU imu;

    private String UAMotorName = "elbow";
    private String LAMotorName = "wrist";
    private String ClawServoName = "claw";
    private String IntakeServoName = "intake";

    // UpperArm predefined target positions
    private static int UA_POS_MIN = 0;
    private static int UA_POS_MAX = 4650;
    private static int UA_POS_GET_SPECIMEN = 0;
    private static int UA_POS_GET_SAMPLE = 230;
    private static int UA_POS_BL_HOVER = 3500;
    private static int UA_POS_CL_HOVER = 2700;
    private static int UA_POS_CL_CLIP = 2100;
    private static int UA_POS_CH_HOVER = 4500;
    private static int UA_POS_CH_CLIP = 3900;
    private static int UA_POS_RL_HOVER = 4400;
    private static int UA_POS_RL_LOCK = 4300;  //UNTESTED

    // LowerArm predefined target positions
    private static int LA_POS_MIN = 0;
    private static int LA_POS_MAX = 230;
    private static int LA_POS_SPECIMEN = 0;
    private static int LA_POS_SAMPLE = 215;
    private static int LA_POS_BL_HOVER = 230;

    // Claw predefined positions
    private static double CLAW_OPEN_POSITION = 0.5;
    private static double CLAW_CLOSED_POSITION = 1.0;

    private int targetUpperArm, targetLowerArm;
    private static double lastUAPosActual, totalUAPosInt=0;

    // Initialize Class
    public ActuatorTools(HardwareMap hardwareMap) {
        // Map the hardware actuator variables.
        UAMotor = hardwareMap.get(DcMotor.class, UAMotorName);
        LAMotor = hardwareMap.get(DcMotor.class, LAMotorName);
        ClawServo = hardwareMap.get(Servo.class, ClawServoName);
        IntakeServo = hardwareMap.get(CRServo.class, IntakeServoName);
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        )));
    }

    public class LowBasketHover implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int UAPos = UAMotor.getCurrentPosition();
            int UATgt = UA_POS_BL_HOVER;

            if (!initialized) {
                UAMotor.setTargetPosition(UATgt);
                LAMotor.setTargetPosition(LA_POS_BL_HOVER);
                UAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LAMotor.setPower(1);
                lastUAPosActual = UAPos;
                initialized = true;
            }

            double UAAng = Angle(UAPos,UA_rPt,UA_pOt);
            UAMotor.setPower( Power(UAPos, UATgt, 0, UA_kP, UA_kI, UA_kD, UA_kA, UAAng) );
            packet.put("UAMotorPos", UAPos);
            return Math.abs(UAPos- UATgt)<1;
        }
    }

    // UpperArm PIDF Coefficients
    private static double UA_kA=0;      // Must be less than 1.
    private static double UA_kP=0;
    private static double UA_kI=0;
    private static double UA_kD=0;
    private static double UA_rPt = 2*Math.PI/(28.0*2.89*3.61*5.23*(90.0/45.0)*(125.0/45.0));  //Radians per Tick
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

