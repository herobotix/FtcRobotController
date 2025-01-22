package org.firstinspires.ftc.team22256.methods;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.view.ContextThemeWrapper;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team22256.methods.arm;
public class drive{
    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor rotator;
    private DcMotor slide;
    private PIDController controller0;
    public static double p=0,i=0,d=0;
    public static double f = 0;
    double pid = 0;
    double ff = 0;
    double power = 0;
    public static double target = 0;
    public static double ticks_in_degree = 4.687;



    public drive(DcMotor fL,DcMotor fR,DcMotor bL,DcMotor bR){
        fR  = hardwareMap.get(DcMotor.class,"rightFront");
        fL  = hardwareMap.get(DcMotor.class,"leftFront");
        bR = hardwareMap.get(DcMotor.class,"rightBack");
        bL = hardwareMap.get(DcMotor.class,"leftBack");
    }
}