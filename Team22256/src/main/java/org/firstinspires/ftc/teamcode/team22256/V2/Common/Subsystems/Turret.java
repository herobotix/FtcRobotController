package org.firstinspires.ftc.teamcode.team22256.V2.Common.Subsystems;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Turret implements Subsystem {

    public static final Turret INSTANCE = new Turret();
    private Turret() { }

    // internal variables to teh turret

    private MotorEx turret = new MotorEx("turret")
            .zeroed()
            .brakeMode();


    public static  double kV = -0.0135;
    public static  double kP = -0.0007;
    public static double kI = 0;
    public static double kD = 0;
    private final double TICKS_PER_DEGREE = (double) 404 / 360;
    private final double TICKS_PER_REVOLUTION = TICKS_PER_DEGREE * 360;
    public static double MAX_POWER = 0.5;
    private double startingPosition = 0;
    private double output = 0;
    private double rightLimit = 0;
    private double leftLimit = 0;

    public enum State {
        VISION,
        HOME,
        IDLE
    }
    private State state = State.IDLE;


    public void setTurretState(State state1){
        state = state1;
    }
    public State getTurretState(){
        return state;
    }

    PIDFController controller = new PIDFController(kP,kI,kD,kV);


    private double A2T(double degrees){
        return degrees * TICKS_PER_DEGREE;
    }
    public double getCurrentPosition(){
        return turret.getRawTicks();
    }

    public double getStartingPosition() {
        return startingPosition;
    }

    public void setStartingPosition(double startingPosition) {
        this.startingPosition = startingPosition;
    }

    public Command setState(State state){
        return new InstantCommand(() -> setTurretState(state));
    }
    public void setUpTurret(){
        startingPosition = getCurrentPosition();

        rightLimit = startingPosition - 130;
        leftLimit = startingPosition + 90;
    }

    @Override
    public void initialize() {


    }



    @Override
    public void periodic() {
        double currentPosition = getCurrentPosition();

        switch (state) {
            case VISION:
            double tx = Limelight.INSTANCE.getTx();
            if (Limelight.INSTANCE.targetFound()) {
                controller.setPIDF(kP, kI, kD, kV);
                controller.setSetPoint(A2T(tx));
                if (currentPosition <= rightLimit) {
                    output = controller.calculate(currentPosition, startingPosition);
                } else if (currentPosition >= leftLimit) {
                    output = controller.calculate(currentPosition, startingPosition);
                } else {
                    output = controller.calculate(currentPosition);
                }
            } else {
                output = 0;
            }

            turret.setPower(output);
            break;

            case IDLE:
                turret.setPower(0);
                break;

            case HOME:
                double error = startingPosition  - Math.abs(currentPosition);
                output = kP * error + kV * startingPosition;
                turret.setPower(output);
                ActiveOpMode.telemetry().addData("error",controller.getPositionError());
                ActiveOpMode.telemetry().addData("output",output);
                ActiveOpMode.telemetry().addData("current position",currentPosition);
                ActiveOpMode.telemetry().addData("starting position",getStartingPosition());
                ActiveOpMode.telemetry().addData("error",error);
                break;
        }
    }
}



