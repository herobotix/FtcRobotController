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

    public static final Turret INSTANCE = new Turret();
    private Turret() { }

    // internal variables to teh turret
    private ControlSystem turretController;
    private MotorEx turret = new MotorEx("turret")
            .zeroed()
            .brakeMode();

    private double kP = 0.01;
    private double kI = 0.000;
    private double kD = 0.0005;

    private double target = 0;
    private double lastError = 0;
    private double angleTolerance = 0.5;

    private double MAX_POWER = 0.25;
    private double turretOutput = 0;
    private ElapsedTime timer = new ElapsedTime();

    private double currentPosition = 0;
    private double startingPosition = 0;

    private double rightLimit;
    private double leftLimit;
    private double lastKnownPosition;
    private double error;

    private PIDController controller0 = new PIDController(kP, kI, kD);

    public enum State {
        VISION,
        PREDICTION,
        IDLE
    }

    private State state = State.VISION;
    private final double TICKS_PER_REV = 404;
    private final double TICKS_PER_DEGREE = TICKS_PER_REV / 360;

    private double a2T(double degrees) {
        return degrees * TICKS_PER_DEGREE;
    }

    public boolean isAiming() {
        return state == State.VISION;
    }

    public void setState(State newState) {
        state = newState;
    }

    public void stopTurret() {
        state = State.IDLE;
        turret.setPower(0.0);
    }

    public double getRawTicks() {
        return turret.getRawTicks();
    }

    public double getSetPoint() {
        return controller0.getSetPoint();
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

            if (Limelight.INSTANCE.targetFound()) {
                lastKnownPosition = currentPosition;
                double tx = Limelight.INSTANCE.getTx();
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


