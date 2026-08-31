
package org.firstinspires.ftc.teamcode.team22256.V2.OpMode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team22256.V2.Common.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.team22256.V2.Common.Subsystems.Sorter;
import org.firstinspires.ftc.teamcode.team22256.V2.Common.Subsystems.Turret;

import java.util.concurrent.ScheduledFuture;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous

public class BlueAuto extends NextFTCOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor backLeft,backRight,frontLeft,frontRight;



    public boolean isScheduled = false;


    public Command shoot = new SequentialGroup(
            Sorter.INSTANCE.BK_UpDown(),
            new Delay(0.3),
            Sorter.INSTANCE.LK_UpDown(),
            new Delay(0.3),
            Sorter.INSTANCE.RK_UpDown()
    );


    public BlueAuto(){
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE,Sorter.INSTANCE,Turret.INSTANCE)
        );
    }



    public Command auto = new SequentialGroup(
            Shooter.INSTANCE.farTriangle,
            new Delay(2),
            shoot,
            shoot
    );



    @Override
    public void onInit(){
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        Turret.INSTANCE.setTurretState(Turret.State.IDLE);

        Shooter.INSTANCE.setShooterVelocity(0).schedule();

    }
    @Override
    public void onStartButtonPressed(){

        Turret.INSTANCE.setTurretState(Turret.State.VISION);

        runtime.reset();

        Shooter.INSTANCE.setShooterVelocity(0).schedule();



    }

    @Override
    public void onUpdate(){

        if(!isScheduled){
            auto.schedule();
            isScheduled = true;
        }

        while(runtime.seconds() > 6 && runtime.seconds() < 7.5 ){
            backLeft.setPower(-0.5);//forward
            backRight.setPower(-0.5);//forward
            frontRight.setPower(-0.5);//forward
            frontLeft.setPower(-0.5);//forward
        }
        backLeft.setPower(0);//forward
        backRight.setPower(0);//forward
        frontRight.setPower(0);//forward
        frontLeft.setPower(0);//forward


        telemetry.addData("timer",runtime.seconds());
        telemetry.update();




    }

}