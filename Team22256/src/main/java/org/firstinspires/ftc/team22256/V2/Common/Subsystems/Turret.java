package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Turret implements Subsystem {

    public static ControlSystem turretController;
    public static final Turret INSTANCE = new Turret();

    private Turret() {

    }
    public static MotorEx turret = new MotorEx("turret")
            .zeroed()
            .brakeMode();
    public static double kP = 0.01;
    public static double kI = 0.000;
    public static double kD = 0.0005;
    public double target = 0;
    public static double lastError = 0;
    public double angleTolerance = 0.5;
    public double MAX_POWER = 0.25;
    public double turretOutput = 0;
    public ElapsedTime timer = new ElapsedTime();
    private static double currentPosition = 0;
    private static double startingPosition = 0;
    double rightLimit;
    double leftLimit;
    double lastKnownPosition;
    double error;

    public static PIDController controller0 = new PIDController(kP, kI, kD);

    public static enum State {
        VISION,
        PREDICTION,
        IDLE
    }

    public static State state = State.VISION;
    private static final double TICKS_PER_REV = 404;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360;



    public static double a2T(double degrees) {
        return degrees * TICKS_PER_DEGREE;
    }

    public static boolean isAiming() {
        return state == State.VISION;
    }

    public static void stopTurret() {
        state = State.IDLE;
        turret.setPower(0.0);
    }



    @Override
    public void initialize() {

        startingPosition = turret.getCurrentPosition();
        currentPosition = startingPosition;
        lastKnownPosition = startingPosition;
        target = startingPosition;


        turretOutput = 0;
        rightLimit = startingPosition - 80;
        leftLimit = startingPosition + 98;

        controller0.setTolerance(7);
    }

    @Override
    public void periodic() {

        currentPosition = turret.getCurrentPosition();

        if (state == State.VISION) {







            if (Limelight.targetFound()) {
                lastKnownPosition = currentPosition;
                double tx = Limelight.getTx();
                 error = a2T(tx) * 2;
                target = currentPosition - error;
                if (target <= rightLimit) {
                    target = rightLimit + 5;
                } else if (target >= leftLimit) {
                    target = leftLimit + 5;

                }
                turretOutput = controller0.calculate(currentPosition, target);

            } else {
                turret.setPower(0.3);
                state = State.IDLE;
                turretOutput = controller0.calculate(currentPosition, lastKnownPosition);
            }
            turretOutput = Range.clip(turretOutput,-MAX_POWER,MAX_POWER);
            turret.setPower(turretOutput);

        } else {
            turret.setPower(0);
        }



    }
}


