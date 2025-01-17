package org.firstinspires.ftc.team22256.tele_op;
import android.os.Environment;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import java.io.File;
import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.HashMap;
import java.util.List;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

@TeleOp
public class Record_Autonomous extends LinearOpMode {

    String FILENAME = "YOUR AUTO NAME HERE";

    int recordingLength = 30;

    //List of each "Frame" of the recording | Each frame has multiple saved values that are needed to fully visualize it
    ArrayList<HashMap<String, Double>> recording = new ArrayList<>();

    final ElapsedTime runtime = new ElapsedTime();
    boolean isPlaying = false;
    int frameCounter = 0;
    int robotState = 0;
    private DcMotor slide;
    private Servo claw;
    private Servo wrist;
    private DcMotor arm1;
    private DcMotor arm2;
    private PIDController controller0; // Declaration of controller object
    private PIDController controller1;
    //  [] 0 is for arm. [] 1 is for extension
    public static double p[] = {-0.041,0.01};
    public static double i[] = {0,0};
    public static double d[] = {0,0};
    public static double f[] = {-0.003,-0.07};
    double pid[] = {0,0};
    double ff[] = {0,0};
    double power[] = {0,0};
    public static double target[] = {0,0};

    public static double ticks_in_degree[] = {0.8,4.687}; // Counts per revoluton / 360
    double changePower = 1.25;

    //Variables for motor usage
    DcMotor leftFront, leftBack, rightFront, rightBack; //Declaring variables used for motors

    @Override
    public void runOpMode() {

        //Increasing efficiency in getting data from the robot
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //Attaching the variables declared with the physical motors by name or id
        {
            leftFront = hardwareMap.dcMotor.get("leftFront");
            leftBack = hardwareMap.dcMotor.get("leftBack");
            rightFront = hardwareMap.dcMotor.get("rightFront");
            rightBack = hardwareMap.dcMotor.get("rightBack");
            arm1 = hardwareMap.get(DcMotor.class, "arm1");
            arm2 = hardwareMap.get(DcMotor.class, "arm2");
            slide = hardwareMap.get(DcMotor.class, "slide");
            claw = hardwareMap.get(Servo.class,"claw");
            wrist = hardwareMap.get(Servo.class,"wrist");

        }

        telemetry.addData("Status", "Waiting to Start");
        telemetry.update();

        waitForStart();
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        target[1] = 0;
        target[0] = -10;

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                //Before recording, gives driver a moment to get ready to record
                //Once the start button is pressed, recording will start
                if (gamepad1.start && robotState == 0) {
                    robotState = 1;
                    runtime.reset();
                    telemetry.addData("Status", "Recording");
                    telemetry.addData("Time until recording end", recordingLength - runtime.time() + "");
                }
                else if(robotState == 0){
                    telemetry.addData("Status", "Waiting to start recording");
                    telemetry.addData("Version", "1");
                }

                //The recording has started and inputs from the gamepad are being saved in a list
                else if(robotState == 1){
                    if(recordingLength - runtime.time() > 0){
                        telemetry.addData("Status", "Recording");
                        telemetry.addData("Time until recording end", recordingLength - runtime.time() + "");
                        HashMap<String, Double> values = robotMovement();
                        recording.add(values);
                    }else{
                        robotState = 2;
                    }
                }

                //PAUSE BEFORE REPLAYING RECORDING
                //Reset the robot position and samples

                //Press START to play the recording
                else if(robotState == 2){
                    telemetry.addData("Status", "Waiting to play Recording" + recording.size());
                    telemetry.addData("Time", runtime.time() + "");
                    if (gamepad1.start){
                        runtime.reset();
                        robotState = 3;
                        telemetry.addData("Status", "Playing Recording");
                        telemetry.update();
                        isPlaying = true;
                        playRecording(recording);
                    }
                }

                //Play-back the recording(This is very accurate to what the autonomous will accomplish)
                //WARNING: I recommend replaying the recording(What was driven and what was replayed vary a lot!)

                //Press the X button to stop(The recording does not stop on its own)
                else if(robotState == 3){
                    if(gamepad1.x){
                        isPlaying = false;
                    }
                    if(isPlaying){
                        playRecording(recording);
                    }else{
                        robotState = 4;
                        telemetry.addData("Status", "Done Recording play-back");
                        telemetry.addData("Save to file", "Press start to save");
                        telemetry.update();
                    }
                }

                //Press START one last time to save the recording
                //After you see the confirmation, you may stop the program.
                else if(robotState == 4){
                    if(gamepad1.start){
                        telemetry.addData("Status", "Saving File");
                        boolean recordingIsSaved = false;
                        String path = String.format("%s/FIRST/data/" + FILENAME + ".fil",
                                Environment.getExternalStorageDirectory().getAbsolutePath());



                        telemetry.clearAll();
                        telemetry.addData("Status", saveRecording(recording, path));
                        telemetry.update();
                    }
                }

                telemetry.update();
            }
        }
    }

    //Writes the recording to file
    public String saveRecording(ArrayList<HashMap<String, Double>> recording, String path){
        String rv = "Save Complete";

        try {
            File file = new File(path);

            FileOutputStream fos = new FileOutputStream(file);
            ObjectOutputStream oos = new ObjectOutputStream(fos);

            oos.writeObject(recording);
            oos.close();
        }
        catch(IOException e){
            rv = e.toString();
        }

        return rv;
    }

    //Think of each frame as a collection of every input the driver makes in one moment, saved like a frame in a video is
    private void playRecording(ArrayList<HashMap<String, Double>> recording){
        //Gets the correct from from the recording

        //The connection between the robot and the hub is not very consistent, so I just get the inputs from the closest timestamp
        //and use that
        double largestTime = 0;
        int largestNum = 0;
        int correctTimeStamp = 0;
        for(int i = 0; i < recording.size();i++){
            if(recording.get(i).get("time") > largestTime){
                if(recording.get(i).get("time") <= runtime.time()){
                    largestTime = recording.get(i).get("time");
                    largestNum = i;
                }
                else{
                    correctTimeStamp = largestNum;
                }
            }
        }
        //Only used inputs are saved to the final recording, the file is too large if every single timestamp is saved.
        telemetry.addData("correctTimeStamp", correctTimeStamp + "");
        telemetry.update();
        HashMap<String, Double> values = recording.get(correctTimeStamp);

        double forwardBackwardValue = values.getOrDefault("rotY", 0.0);
        double leftRightValue = values.getOrDefault("rotX", 0.0);
        double turningValue = values.getOrDefault("rx", 0.0);

        double highestValue = Math.max(Math.abs(forwardBackwardValue) + Math.abs(leftRightValue) + Math.abs(turningValue), 1);

        //Calculates amount of power for each wheel to get the desired outcome
        //E.G. You pressed the left joystick forward and right, and the right joystick right, you strafe diagonally while at the same time turning right, creating a circular strafing motion.
        //E.G. You pressed the left joystick forward, and the right joystick left, you drive like a car and turn left
        if(highestValue >= 0.1){
            leftFront.setPower((forwardBackwardValue + leftRightValue + turningValue) / highestValue);
            leftBack.setPower((forwardBackwardValue - leftRightValue + turningValue) / highestValue);
            rightFront.setPower((forwardBackwardValue - leftRightValue - turningValue) / highestValue);
            rightBack.setPower((forwardBackwardValue + leftRightValue - turningValue) / highestValue);
        }
    }

    //Simple robot movement
    //Slowed to half speed so movements are more accurate
    private HashMap<String, Double> robotMovement() {
        frameCounter++;
        HashMap<String, Double> values = new HashMap<>();
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        telemetry.addData("left stick x", gamepad1.left_stick_x);
        telemetry.addData("left stick y", gamepad1.left_stick_y);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower = frontLeftPower / changePower;
        backLeftPower = backLeftPower / changePower;
        frontRightPower = frontRightPower / changePower;
        backRightPower = backRightPower / changePower;

        leftFront.setPower(-frontLeftPower);
        leftBack.setPower(-backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(-backRightPower);


        if(gamepad1.right_trigger > 0.8){
            target[0] = -10;
            sleep(500);
            arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }



        if(gamepad1.right_bumper){
            wrist.setPosition(0.1);

        } else {
            wrist.setPosition(0.6);
        }
        if(gamepad1.left_bumper){
            claw.setPosition(0.65
            );
        } else {
            claw.setPosition(0.1);
        }



        int armPos = arm1.getCurrentPosition();
        pid[0] = controller0.calculate(armPos,target[0]);
        ff[0] = Math.cos(Math.toRadians(target[0] / ticks_in_degree[0])) * f[0];
        power[0] = pid[0] + ff[0];
        arm1.setPower(-power[0]);
        arm2.setPower(power[0]);

        if(gamepad1.dpad_up && target[0] < 150 && target[0] >= 0){
            target[0] = target[0] + 7;
        }else if(gamepad1.dpad_up && target[0] <= 300 && target[0] >= 150) {
            target[0] = target[0] + 3.5;  //up for arm
        }else if(gamepad1.dpad_down && target[0] >=150 && target[0] <= 300){
            target[0] = target[0] - 6;
        }else if(gamepad1.dpad_down && target[0] >= 0 && target[0] <= 150){
            target[0] = target[0] - 4;  //down for arm
        }


        if(target[0] >= 300){
            target[0] = 300;
        }
        if(target[0] < 0){
            target[0] = 0;
        }
        if(target[0] < 240 && target[1] > 2500 ){
            target[0] = 240;
        }


        int SPos = slide.getCurrentPosition();
        pid[1] = controller1.calculate(SPos,target[1]);
        ff[1] = Math.cos(Math.toRadians(target[1] / ticks_in_degree[1])) * f[1];
        power[1] = pid[1] + ff[1];
        slide.setPower(power[1]);


        if (gamepad1.x && target[0] >= 0 && target[0] <= 50){
            target[1] = target[1] - 30;
        }else if (gamepad1.y && target[0] >= 0 && target[0] <= 50){
            target[1] = target[1] + 30;
        }

        if(target[0] <= 0 && target[0] >= -30 && target[1] <= -1021){
            target[1] =-1020;
        }



        if (gamepad1.x && target[0] <= 300 && target[0] > 50){
            target[1] = -3300;
        }else if (gamepad1.y && target[0] <= 300 && target[0] > 50){
            target[1] = -10;
        }




        return values;
    }
}