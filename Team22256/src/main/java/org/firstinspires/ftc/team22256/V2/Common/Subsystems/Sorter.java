package org.firstinspires.ftc.team22256.V2.Common.Subsystems;

import static org.firstinspires.ftc.team22256.V2.Common.Global.motif;

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

import java.util.ArrayList;
import java.util.List;

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
    private Sorter() { }

    private ServoEx LK = new ServoEx("l-kicker");
    private ServoEx RK = new ServoEx("r-kicker");
    private ServoEx BK = new ServoEx("b-kicker");

    private NormColorSensor[][] sensors = new NormColorSensor[3][2]; //3 ball slots;average the value from the 2 color sensor per spot
    private ServoEx[] kickers = new ServoEx[3];// 1 kicker per ball slot
    public NormColorSensor.COLOR[] storedColors = new NormColorSensor.COLOR[3];//1 color per ball slot
    public boolean[] scheduled = {false,false,false};

    private final double LKDown = 0.025;
    private final double LKUp = 0.45;
    private final double RKDown = 0.95;
    private final double RKUp = 0.55;
    private final double BKDown = 0.66;
    private final double BKUp = 0;

    public Command LK_Up = new InstantCommand(() -> LK.setPosition(LKUp));
    public Command LK_Down = new InstantCommand(() -> LK.setPosition(LKDown));
    public Command RK_Up = new InstantCommand(() -> RK.setPosition(RKUp));
    public Command RK_Down = new InstantCommand(() -> RK.setPosition(RKDown));
    public Command BK_Up = new InstantCommand(() -> BK.setPosition(BKUp));
    public Command BK_Down = new InstantCommand(() -> BK.setPosition(BKDown));
    public Command LK_UpDown(){
        return new SequentialGroup(LK_Up,new Delay(0.3), LK_Down);
    }
    public Command RK_UpDown(){
        return new SequentialGroup(RK_Up,new Delay(0.3), RK_Down);
    }
    public Command BK_UpDown(){
        return new SequentialGroup(BK_Up,new Delay(0.3), BK_Down);
    }

    public NormColorSensor.COLOR updateSpotColor(int spot) {
        float normRed = (sensors[spot][0].getRed() + sensors[spot][1].getRed())/2;
        float normGreen = (sensors[spot][0].getGreen() + sensors[spot][1].getGreen())/2;
        float normBlue = (sensors[spot][0].getBlue() + sensors[spot][1].getBlue())/2;
        if(normBlue > normGreen && normRed > (0.5 * normGreen) && normRed > 0.01){
            return NormColorSensor.COLOR.PURPLE; //purple
        } else if(normGreen > 0.03 && normGreen > (normRed + 0.01) && normGreen > (normBlue + 0.003)){
            return NormColorSensor.COLOR.GREEN; //green
        } else{
            return NormColorSensor.COLOR.EMPTY;//none
        }
    }

    //0 = left, 1 = right, 2 = back
    public void updateAllSpots() {
        for(int i = 0; i < 3;i++){
            storedColors[i] = updateSpotColor(i);
        }
    }

    @Override
    public void initialize() {
        sensors[0][0] = new NormColorSensor(ActiveOpMode.hardwareMap(),"clr-2");
        sensors[0][1] = new NormColorSensor(ActiveOpMode.hardwareMap(),"clr-3");
        sensors[1][0] = new NormColorSensor(ActiveOpMode.hardwareMap(),"clr-5");
        sensors[1][1] = new NormColorSensor(ActiveOpMode.hardwareMap(),"clr-6");
        sensors[2][0] = new NormColorSensor(ActiveOpMode.hardwareMap(),"clr-1");
        sensors[2][1] = new NormColorSensor(ActiveOpMode.hardwareMap(),"clr-4");
    }

    @Override
    public void periodic() {
        //sensors[0][0].updateColors();//updates values from sensors
        //sensors[0][1].updateColors();
        //sensors[1][0].updateColors();
        //sensors[1][1].updateColors();
        //sensors[2][0].updateColors();
        //sensors[2][1].updateColors();
    }
}