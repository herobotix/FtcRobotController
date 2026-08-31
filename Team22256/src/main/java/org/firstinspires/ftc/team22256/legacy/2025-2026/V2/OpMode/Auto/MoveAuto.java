
package org.firstinspires.ftc.team22256.V2.OpMode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Shooter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Sorter;
import org.firstinspires.ftc.team22256.V2.Common.Subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous

public class MoveAuto extends NextFTCOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor backLeft,backRight,frontLeft,frontRight;






    public Command shoot = new SequentialGroup(
            Sorter.INSTANCE.BK_UpDown(),
                        new Delay(0.3),
                        Sorter.INSTANCE.LK_UpDown(),
                                new Delay(0.3),
                        Sorter.INSTANCE.RK_UpDown()
                                );


public MoveAuto(){
    addComponents(
            new SubsystemComponent(Shooter.INSTANCE,Sorter.INSTANCE)
    );
}


    @Override
    public void runOpMode() {


        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        runtime.reset();
        waitForStart();

        if (opModeIsActive()) {

            Shooter.INSTANCE.closeTriangle.schedule();
            runtime.reset();


            while(runtime.seconds() < 0.3){
                backLeft.setPower(-1);//forward
                backRight.setPower(1);//forward
                frontRight.setPower(-1);//forward
                frontLeft.setPower(1);//forward
            }
            //shoot.schedule();

            telemetry.update();
            stop();

        }
    }
}
