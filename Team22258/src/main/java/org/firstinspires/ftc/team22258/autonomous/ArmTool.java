package org.firstinspires.ftc.team22258.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ArmTool {
    private DcMotor Arm;
    private Servo LClaw, RClaw;
    
    private String ArmName = "Arm";
    private String LClawName = "LClaw";
    private String RClawName = "RClaw";
    
    // Arm Positions
    public static int armMin = 0;
    public static int armMax = 0;
    
    // Claw predefined positions
    public static double clawLO = 0.25;
    public static double clawLC = 0.75;
    public static double clawRO = 0.75;
    public static double clawRC = 0.25;
    
    public static double kA = 0.0009;  // Must be less than 1.  % of power, maximum torque with arm horizontal.
    public static double kS = 0.1;
    public static double kP = 1.0;
    public static double GearRatio = 3.61 * 3.61 * 5.23 * 4.00;     // 4:1, 4:1, 5:1 Cartridges, 4:1 Sprocket/Chain
    public static double RpT = 2 * Math.PI / (28.0 * GearRatio);    //Radians per Tick
    public static double HPiT = 880;                                //Horizontal Position in Ticks
    public static int toleranceArmPos = 10;
    
    public int currentArmPos = 0;
    public static double initArmPos;
    
    // Initialize Class
    public ArmTool(HardwareMap hardwareMap, String toolName) {
        Arm = hardwareMap.get(DcMotor.class, toolName);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        LClaw = hardwareMap.get(Servo.class, LClawName);
        RClaw = hardwareMap.get(Servo.class, RClawName);
    }
    
    public Action ArmTo(int targetArmPos) {
        return new Action() {
            private boolean initialized = false;
            
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                currentArmPos = Arm.getCurrentPosition();
                packet.put("CurrentPos", currentArmPos);
                packet.put("TargetPos", targetArmPos);
                double angArmPos = Angle(currentArmPos);
                packet.put("CurrentAngle", angArmPos);
                
                if (!initialized) {
                    Arm.setTargetPosition(targetArmPos);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    initArmPos = currentArmPos;
                    initialized = true;
                }
                
                Arm.setPower(Power(currentArmPos, targetArmPos, angArmPos));
                return Math.abs(currentArmPos - targetArmPos) > toleranceArmPos;
            }
            
            // Arm PIDF Coefficients
            public double Angle(int curPos) { return ((RpT * (curPos - HPiT)) + (Math.PI / 2)); }
            public double Power(int curPos, int tgtPos, double angPos) {
                double DTot = tgtPos - initArmPos, DCur = tgtPos - curPos;
                if (Math.abs(curPos - tgtPos) < toleranceArmPos) { return kA * Math.sin(angPos); }
                return (kA * Math.sin(angPos)) + ((1 - kA) * Math.signum(DCur));
                //return (kA * Math.sin(angPos)) + ((1 - kA) * Math.signum(DCur) * (kS + kP*Math.sin(DCur*Math.PI/DTot)));
            }
        };
        
    }
    
    public Action ClawState(boolean Open) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                packet.put("ClawPos", Open ? "Open" : "Closed");
                if (Open) {
                    LClaw.setPosition(clawLO);
                    RClaw.setPosition(clawRO);
                } else {
                    LClaw.setPosition(clawLC);
                    RClaw.setPosition(clawRC);
                }
                return false;
            }
        };
        
    }
}