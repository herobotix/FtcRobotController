package org.firstinspires.ftc.team22256.methods;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

public class arm{

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

    public void topPos(){

        target = -4000;

    }
    public void bottomPos(){

        target = -100;

    }


    }


