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
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;


public class Sorter implements Subsystem {
    public static final Sorter INSTANCE = new Sorter();
    private Sorter(){
        LKSL = new NormColorSensor(ActiveOpMode.hardwareMap(),"String");
        LKSR = new NormColorSensor(ActiveOpMode.hardwareMap(),"String");
        RKSL = new NormColorSensor(ActiveOpMode.hardwareMap(),"String");
        RKSR = new NormColorSensor(ActiveOpMode.hardwareMap(),"String");
        BKSL = new NormColorSensor(ActiveOpMode.hardwareMap(),"String");
        BKSR = new NormColorSensor(ActiveOpMode.hardwareMap(),"String");
    }

    private ServoEx LK = new ServoEx("l-kicker");
    private ServoEx RK = new ServoEx("r-kicker");
    private ServoEx BK = new ServoEx("b-kicker");
    private NormColorSensor LKSL;//left kicker sensor left
    private NormColorSensor LKSR;//left kicker sensor right
    private NormColorSensor RKSL;//right kicker sensor left
    private NormColorSensor RKSR;//right kicker sensor right
    private NormColorSensor BKSL;//back kicker sensor left
    private NormColorSensor BKSR;//back kicker sensor right
    private final double LKDown = 0;
    private final double LKUp = 0;
    private final double RKDown = 0;
    private final double RKUp = 0;
    private final double BKDown = 0;
    private final double BKUp = 0;
    int LKSLC;//left kicker sensor left color
    int LKSRC;//left kicker sensor right color
    int RKSLC;//right kicker sensor left color
    int RKSRC;//right kicker sensor right color
    int BKSLC;//back kicker sensor left color
    int BKSRC;//back kicker sensor right color


public Command LK_Up = new LambdaCommand()
        .setStart(() -> LK.setPosition(LKUp))
        .setIsDone(() -> true)
        .requires(this);

    public Command LK_Down = new LambdaCommand()
            .setStart(() -> LK.setPosition(LKDown))
            .setIsDone(() -> true)
            .requires(this);

    public Command RK_Up = new LambdaCommand()
            .setStart(() -> RK.setPosition(RKUp))
            .setIsDone(() -> true)
            .requires(this);
    public Command RK_Down = new LambdaCommand()
            .setStart(() -> RK.setPosition(RKDown))
            .setIsDone(() -> true)
            .requires(this);
    public Command BK_Up = new LambdaCommand()
            .setStart(() -> BK.setPosition(BKUp))
            .setIsDone(() -> true)
            .requires(this);
    public Command BK_Down = new LambdaCommand()
            .setStart(() -> BK.setPosition(BKDown))
            .setIsDone(() -> true)
            .requires(this);

    @Override
    public void periodic() {
        LKSLC = LKSL.getDetectedColor();
        LKSRC = LKSR.getDetectedColor();
        RKSLC = RKSL.getDetectedColor();
        RKSRC = RKSR.getDetectedColor();
        BKSLC = BKSL.getDetectedColor();
        BKSRC = BKSR.getDetectedColor();
    }
}