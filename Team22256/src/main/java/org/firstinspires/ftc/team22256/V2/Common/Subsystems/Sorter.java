package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.NormColorSensor;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.core.commands.groups.SequentialGroup;


public class Sorter implements Subsystem {
    public static final Sorter INSTANCE = new Sorter();
    private Sorter(){
    }
    int[] colorPositions = {0,0,0,0,0,0};
    private static ServoEx LK = new ServoEx("l-kicker");
    private static ServoEx RK = new ServoEx("r-kicker");
    private static ServoEx BK = new ServoEx("b-kicker");
    private NormColorSensor LKSL;//left kicker sensor left
    private NormColorSensor LKSR;//left kicker sensor right
    private NormColorSensor RKSL;//right kicker sensor left
    private NormColorSensor RKSR;//right kicker sensor right
    private NormColorSensor BKSL;//back kicker sensor left
    private NormColorSensor BKSR;//back kicker sensor right
    private static final double LKDown = 0.025;
    private static final double LKUp = 0.4;
    private static final double RKDown = 0.95;
    private static final double RKUp = 0.6;
    private static final double BKDown = 1;
    private static final double BKUp = 0;
    int LKSLC;//left kicker sensor left color
    int LKSRC;//left kicker sensor right color
    int RKSLC;//right kicker sensor left color
    int RKSRC;//right kicker sensor right color
    int BKSLC;//back kicker sensor left color
    int BKSRC;//back kicker sensor right color
    public static Command LK_Up = new InstantCommand(() -> LK.setPosition(LKUp));
    public static Command LK_Down = new InstantCommand(() -> LK.setPosition(LKDown));
    public static Command RK_Up = new InstantCommand(() -> RK.setPosition(RKUp));
    public static Command RK_Down = new InstantCommand(() -> RK.setPosition(RKDown));
    public static Command BK_Up = new InstantCommand(() -> BK.setPosition(BKUp));
    public static Command BK_Down = new InstantCommand(() -> BK.setPosition(BKDown));
    public static Command LK_UpDown(){
        return new SequentialGroup(LK_Up,new Delay(0.2), LK_Down);
    }
    public static Command RK_UpDown(){
        return new SequentialGroup(RK_Up,new Delay(0.2), RK_Down);
    }
    public static Command BK_UpDown(){
        return new SequentialGroup(BK_Up,new Delay(0.2), BK_Down);
    }

    @Override
    public void initialize(){
        LKSL = new NormColorSensor(ActiveOpMode.hardwareMap(),"clr-2");
        LKSR = new NormColorSensor(ActiveOpMode.hardwareMap(),"clr-3");
        RKSL = new NormColorSensor(ActiveOpMode.hardwareMap(),"clr-5");
        RKSR = new NormColorSensor(ActiveOpMode.hardwareMap(),"clr-6");
        BKSL = new NormColorSensor(ActiveOpMode.hardwareMap(),"clr-1");
        BKSR = new NormColorSensor(ActiveOpMode.hardwareMap(),"clr-4");
    }

    @Override
    public void periodic() {
        colorPositions[0] = LKSL.getDetectedColor();
        colorPositions[1] = LKSR.getDetectedColor();
        colorPositions[2] = RKSL.getDetectedColor();
        colorPositions[3] = RKSR.getDetectedColor();
        colorPositions[4] = BKSL.getDetectedColor();
        colorPositions[5] = BKSR.getDetectedColor();
    }
}