package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import com.arcrobotics.ftclib.controller.PIDController;


public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    private Turret(){
    }
    private static MotorEx turret = new MotorEx("turret");
    private static double currentPosition = 0;

    private double target = 0;
    private double turretOutput;
    double LLresult;
    double error = 0;
    PIDController controller0 = new PIDController(kP,kI,kD);
    public static enum State{
        VISION,
        PREDICTION,
        IDLE
    }
    private static State state = State.VISION;
    private static final double  TICKS_PER_REV = 418;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360;

    public static final double kP = 0.01;
    public static final double kI = 0;
    public static final double kD = 0;
    private ControlSystem turretController;

    public static double a2T(double degrees){
        return degrees * TICKS_PER_DEGREE;
    }
    public boolean isAiming() {
        return state == State.VISION;
    }
    public static void stopTurret() {
        state = State.IDLE;
        turret.setPower(0.0);
    }


    public static double getTurretPosition() {
        return currentPosition;
    }
    public static boolean isAimed(){
        if(Limelight.getTx() <3.5){
            return true;
        } else {
            return false;
        }
    }

    public static boolean aim(){
        return true;
    }


    @Override
    public void periodic() {








































        
        }
    }


