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
    private MotorEx turret = new MotorEx("Turret");
    private double currentPosition = 0;
    private double target = 0;
    double turretOutput;
    double LLresult;
    double error = 0;
    PIDController controller0 = new PIDController(kP,kI,kD);
    public enum State{
        VISION,
        PREDICTION,
        IDLE
    }
    private State state = State.IDLE;
    private static final double  TICKS_PER_REV = 1;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    private ControlSystem turretController;

    public static double a2T(double degrees){
        return degrees / TICKS_PER_REV;
    }
    public boolean isAiming() {
        return state == State.VISION;
    }
    public void stopTurret() {
        state = State.IDLE;
        turret.setPower(0.0);
    }


    public double getTurretPosition() {
        return currentPosition;
    }




    @Override
    public void periodic() {
        LLresult = Limelight.getTx();
        currentPosition = turret.getCurrentPosition();
        switch (state){
            case VISION:
                if(!Double.isNaN(LLresult)) {
                    target = (a2T(LLresult)) + getTurretPosition();
                    error = a2T(LLresult);
                     turretOutput = controller0.calculate(currentPosition, target);
                } else {
                    turretOutput = 0;
                    state = State.IDLE;
                }
                break;
            case IDLE:
                if(!Double.isNaN(LLresult)){
                    state = State.VISION;
                }
                break;
            case PREDICTION:
                //TODO use pinpoint to predict heading of april tag
        }
    }

}
