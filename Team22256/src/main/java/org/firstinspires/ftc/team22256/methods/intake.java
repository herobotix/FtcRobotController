package org.firstinspires.ftc.team22256.methods;
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
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
public class intake{

    public intake(DcMotor rotator0,Servo rotator1,Servo rotator2,Servo flapper){
        rotator0 = hardwareMap.get(DcMotor.class,"rotator0");
        rotator1 = hardwareMap.get(Servo.class,"rotator1");
        rotator2 = hardwareMap.get(Servo.class,"rotator2");
        flapper = hardwareMap.get(Servo.class,"flapper");
    }


}