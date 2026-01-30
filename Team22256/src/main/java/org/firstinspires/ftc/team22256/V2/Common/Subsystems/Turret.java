package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Turret implements Subsystem {

    public static ControlSystem turretController;
    public static final Turret INSTANCE = new Turret();
    private Turret(){
        controller0 = new PIDController(kP,kI,kD);
    }
    public static MotorEx turret = new MotorEx("turret")
            .zeroed()
            .brakeMode();
    private static double currentPosition = 0;
    private static double startingPosition = 0;
    double rightLimit;
    double leftLimit;

    private double target = 0;
    double turretOutput = 0;

    double LLresult;
    double error = 0;
    public static  double kP = 0.011;
    public static  double kI = 0;
    public static  double kD = 0.001;
    public static PIDController controller0 = new PIDController(kP,kI,kD);
    public static enum State{
        VISION,
        PREDICTION,
        IDLE
    }
    private static State state = State.VISION;
    private static final double  TICKS_PER_REV = 404;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360;




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

    public static Command setGoal(double ticks){
        return new InstantCommand(() -> {
            controller0.setSetPoint(currentPosition + ticks);
        });
    }
    public static double getTurretPosition() {
        return currentPosition;
    }

    @Override
    public void initialize(){
        startingPosition = turret.getCurrentPosition();
        rightLimit = startingPosition -80;
        leftLimit = startingPosition + 98;
    }

    @Override
    public void periodic() {


        if(state == State.VISION){

            if(Limelight.targetFound()){
                double tx = Limelight.getTx();
                double ticks = a2T(tx);
                currentPosition = turret.getCurrentPosition();
                target = currentPosition - ticks;
                if(target <= rightLimit){
                    turret.setPower(0);
                }
                else if( target >= leftLimit){
                    turret.setPower(0);
                } else {
                    turretOutput = controller0.calculate(currentPosition, target);
                    turret.setPower(turretOutput);
                }

               }

            }

        }
    }


