package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.checkerframework.checker.units.qual.degrees;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;

import java.util.List;
import java.util.Objects;

// Use terminal to connect to Control Hub wirelessly:   adb connect 192.168.43.1:5555

@Autonomous(name = "AutonomousRed", group = "FINAL")
public class AutonomousRed extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotor armRight, armLeft, armMiddle;
    public Servo wristRight, wristLeft, wristArm, launcher;

    private static final double MAX_LINVEL = 2200, MAX_LINACC = 1500;
    private static final double MAX_ANGVEL = 2200, MAX_ANGACC = 1000;
    //private static final double MAX_ANGVEL = 135, MAX_ANGACC = 60;
    private static final double MAX_LATVEL = 2200, MAX_LATACC = 1000;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_red.tflite";
    //private static final String[] LABELS = { "belement", };
    //private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    // Drive Ratios for FRONT/BACK LEFT/RIGHT motors.
    Integer cpr = 27;
    double gearRatio = 15.2;
    double diameter = 2.95276;
    double cpi = (cpr * gearRatio) / (Math.PI * diameter);
    double biasLin = 0.98;
    double INCHES_TO_TICKS = (cpi * biasLin);
    double ipd = 95.1/360.0;
    double ANG_BIAS = 1;//1.3;
    double LAT_BIAS = 1.115;
    double DEGREES_TO_TICKS = (INCHES_TO_TICKS * ipd);
    double BOARD_ANGLE = 115.0;
    double ARM_DEGREES_TO_TICKS = 430.0/120.0;
    double WRIST_DEGREES_TO_TICKS = 1.0/180.0;

    IMU imu;

    int spikeNum;
    String location = "InField";
    boolean doPlacePixel = true, checkTfod = true;
    int armPosition;
    double angPos, angVel;

    @Override
    public void init() {
        initTfod();

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft"); //3
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight"); //1
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft"); //2
        backRight = hardwareMap.get(DcMotorEx.class, "backRight"); //0
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        armRight = hardwareMap.get(DcMotor.class, "armRight");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armMiddle = hardwareMap.get(DcMotor.class, "armMiddle");

        wristArm = hardwareMap.get(Servo.class, "wristArm");
        wristRight = hardwareMap.get(Servo.class, "wristRight");
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        launcher = hardwareMap.get(Servo.class, "launcher");

        imu = hardwareMap.get(IMU.class, "imu");
        imuInit();
    }

    //int inchesTest=3;
    @Override
    public void init_loop() {
        if(gamepad1.a) { location = (Objects.equals(location,"InField")) ? "OutField" : "InField"; sleep(100); }
        if(gamepad1.b) { spikeNum = (spikeNum % 3) + 1; sleep(100); checkTfod = false; }
        if(gamepad1.x) { doPlacePixel = !doPlacePixel; sleep(100); }
        //if(gamepad1.left_bumper) inchesTest--;
        //if(gamepad1.right_bumper) inchesTest++;
        //telemetry.addData("Distance Test: ", inchesTest);

        armLeft.setTargetPosition(0);
        armRight.setTargetPosition(0);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Game Alliance:", "Red");
        telemetry.addData("Field Location (A):", location);
        telemetry.addData("SpikeMark Number:", spikeNum);
        telemetry.addData("Placing Pixel on Board (X): ", doPlacePixel);

        //imuTelemetry();
        if (checkTfod) telemetryTfod();
        telemetry.update();
    }

    @Override
    public void start() {
        visionPortal.stopStreaming(); // stops the stream once we click start to save CPU and random detections.

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();

        wristLeft.setPosition(0.45);
        wristRight.setPosition(0.85);

        if(Objects.equals(location, "OutField")) {
            if (spikeNum == 1) {
                moveLat(-12);
                moveLin( 20);
                moveLin( -6);
                moveAng( -180);
                moveLat(-12);
                lowerArm();
                moveLin( -39);
                moveAng( -90);
                moveLin( -95);
                if (doPlacePixel) {
                    moveLat(-24);
                    placePixel();
                    moveLat(24);
                }
                telemetry.addLine("OutField, Red - left");
                telemetry.update();
            } else if (spikeNum == 2) {
                moveLin( 29);
                moveLin( -10);
                moveLat(-12);
                moveAng( -180);  // FOR REVERSE DRIVING MODE
                lowerArm();
                moveLin( -30);
                moveAng( -90);
                moveLin( -91);
                if (doPlacePixel) {
                    moveLat(-24);
                    placePixel();
                    moveLat(24);
                }
                telemetry.addLine("OutField, Red - middle");
                telemetry.update();
            } else { //right
                moveLin(26);
                moveAng(-90);
                moveLin(3);
                moveLin(-15);
                lowerArm();
                moveAng(-90);
                moveLin( -27);
                moveAng(-90);
                moveLin( -95);
                if (doPlacePixel) {
                    moveLat(-24);
                    placePixel();
                    moveLat(24);
                }
                telemetry.addLine("OutField, Red - right");
                telemetry.update();
            }
        } else {
            if (spikeNum == 1) {
                moveLat(12);
                moveLin( 26);
                moveAng( 90);
                moveLin( 15);
                moveLin( -33);
                lowerArm();
                if (doPlacePixel) {
                    moveLat(8);
                    placePixel();
                    moveLat(-32);
                }
                telemetry.addLine("Infield, Red - left");
                telemetry.update();
            } else if (spikeNum == 2) {
                moveLin( 28);
                moveLin( -22);
                lowerArm();
                moveAng( 90);
                moveLin( -30);
                if (doPlacePixel) {
                    moveLat(24);
                    placePixel();
                    moveLat(-24);
                }
                telemetry.addLine("Infield, Red - middle");
                telemetry.update();
            } else { //right
                moveLat(12);
                moveLin( 20);
                moveLin( -14);
                lowerArm();
                moveAng( 90);
                moveLin( -18);
                if (doPlacePixel) {
                    moveLat(18);
                    placePixel();
                    moveLat(-18);
                }
                telemetry.addLine("Infield, Red - right");
                telemetry.update();
            }
        }

        frontLeft.setVelocity(0);
        frontRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);
    }

    @Override
    public void loop() {}

    private void initTfod() {

        tfod = new TfodProcessor.Builder()
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();
        //tfod.setMinResultConfidence(0.75f);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));

        //builder.setCameraResolution(new Size(640, 480));
        //builder.enableLiveView(true);
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);   //MJPEG
        //builder.setAutoStopLiveView(false);

        builder.addProcessor(tfod);
        visionPortal = builder.build();
        //visionPortal.setProcessorEnabled(tfod, false);
    }

    private void telemetryTfod() {
        double x = -1;

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f", x);
            telemetry.addData("- Size", "%.0f", recognition.getWidth());
        }

        if (x >= 250) spikeNum = 2;
        else if (x >= 0) spikeNum = 1;
        else spikeNum = 3;
        telemetry.addData("SpikeMark Number", spikeNum);
    }

    public void imuInit() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void moveLin(double distance) {      //distance in inches
        double MIN_VEL_STEP = MAX_LINVEL/100;
        int MIN_DIST = (int) (Math.pow(MAX_LINVEL,2)/(2*MAX_LINACC));
        int MOVE = (int) (Math.round(distance * INCHES_TO_TICKS));
        int DIST = Math.min(MIN_DIST,Math.abs(MOVE)/2);
        int fL_INIT = frontLeft.getCurrentPosition();
        int fL_TRGT = fL_INIT + MOVE;

        double curVel = Math.signum(distance) * MIN_VEL_STEP;
        int fLPos = fL_INIT;
        int dPos = (int) (Math.pow(curVel,2)/(2*MAX_LINACC));

        double time = runtime.seconds(), dtime=0, curAcc=0, avgAcc=0, minAcc=MAX_LINACC, maxAcc=0, maxVel=0;
        int n=0;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setVelocity(curVel);
        frontRight.setVelocity(curVel);
        backLeft.setVelocity(curVel);
        backRight.setVelocity(curVel);

        while (Math.abs(fL_TRGT - fLPos) > frontLeft.getTargetPositionTolerance()) {
            if ((Math.abs(fLPos - fL_INIT) < DIST) && (Math.abs(fLPos - fL_INIT) > dPos) && (Math.abs(curVel) < MAX_LINVEL)) {
                curVel = Math.signum(curVel)*Math.min(Math.abs(curVel) + MIN_VEL_STEP,MAX_LINVEL);
                if (Math.abs(curVel) < MAX_LINVEL) dPos = (int) (Math.pow(curVel,2)/(2*MAX_LINACC));
                else dPos = (int) (Math.pow(MAX_LINVEL-MIN_VEL_STEP,2)/(2*MAX_LINACC));
                frontLeft.setVelocity(curVel);
                frontRight.setVelocity(curVel);
                backLeft.setVelocity(curVel);
                backRight.setVelocity(curVel);
                dtime = runtime.seconds() - time;
                time = runtime.seconds();
            } else if ((Math.abs(fL_TRGT - fLPos) < DIST) && (Math.abs(fL_TRGT - fLPos) < dPos) && (Math.abs(curVel) > 0)) {
                curVel = Math.signum(curVel)*Math.max(Math.abs(curVel) - MIN_VEL_STEP,0);
                if (curVel == 0) {
                    dPos = -1;
                    curVel = Math.signum(fL_TRGT - fLPos)*MIN_VEL_STEP;
                }
                else {
                    dPos = (int) (Math.pow(curVel,2)/(2*MAX_LINACC));
                }
                frontLeft.setVelocity(curVel);
                frontRight.setVelocity(curVel);
                backLeft.setVelocity(curVel);
                backRight.setVelocity(curVel);
                dtime = - runtime.seconds() + time;
                time = runtime.seconds();
            }
            fLPos = frontLeft.getCurrentPosition();

            if (dtime != 0) {
                minAcc = Math.min(minAcc,curAcc);
                maxAcc = Math.max(maxAcc,curAcc);
                curAcc = MIN_VEL_STEP/dtime;
                avgAcc = ((avgAcc*n)+curAcc)/++n;
            }

            maxVel = Math.max(maxVel,curVel);

            telemetry.addData("Current Position: ", fLPos);
            telemetry.addData("Current Velocity: ", curVel);
            telemetry.addData("Current Acceleration: ", curAcc);
            telemetry.update();
        }

        frontLeft.setVelocity(0);
        frontRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);

        sleep(125);

        telemetry.addData("MIN_DIST: ", MIN_DIST / INCHES_TO_TICKS);
        telemetry.addData("Tolerance: ", frontLeft.getTargetPositionTolerance() / INCHES_TO_TICKS);
        telemetry.addData("Within Tolerance.", "");
        telemetry.addData("Maximum Velocity: ", maxVel);
        telemetry.addData("Minimum Acceleration: ", minAcc);
        telemetry.addData("Average Acceleration: ", avgAcc);
        telemetry.addData("Maximum Acceleration: ", maxAcc);
        telemetry.update();
    }

    public void moveAng(double degrees) {
        double MIN_VEL_STEP = MAX_ANGVEL/100;
        int MIN_DIST = (int) (Math.pow(MAX_ANGVEL,2)/(2*MAX_ANGACC));
        int MOVE = (int) (Math.round(degrees * DEGREES_TO_TICKS));
        int DIST = Math.min(MIN_DIST,Math.abs(MOVE)/2);
        int fL_INIT = frontLeft.getCurrentPosition();
        int fL_TRGT = fL_INIT - MOVE;

        double curVel = Math.signum(degrees) * MIN_VEL_STEP;
        int fLPos = fL_INIT;
        int dPos = (int) (Math.pow(curVel,2)/(2*MAX_ANGACC));

        double time = runtime.seconds(), dtime=0, curAcc=0, avgAcc=0, minAcc=MAX_ANGACC, maxAcc=0, maxVel=0;
        int n=0;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setVelocity(-curVel);
        frontRight.setVelocity(curVel);
        backLeft.setVelocity(-curVel);
        backRight.setVelocity(curVel);

        while (Math.abs(fL_TRGT - fLPos) > frontLeft.getTargetPositionTolerance()) {
            if ((Math.abs(fLPos - fL_INIT) < DIST) && (Math.abs(fLPos - fL_INIT) > dPos) && (Math.abs(curVel) < MAX_ANGVEL)) {
                curVel = Math.signum(curVel)*Math.min(Math.abs(curVel) + MIN_VEL_STEP,MAX_ANGVEL);
                if (Math.abs(curVel) < MAX_ANGVEL) dPos = (int) (Math.pow(curVel,2)/(2*MAX_ANGACC));
                else dPos = (int) (Math.pow(MAX_ANGVEL-MIN_VEL_STEP,2)/(2*MAX_ANGACC));
                frontLeft.setVelocity(-curVel);
                frontRight.setVelocity(curVel);
                backLeft.setVelocity(-curVel);
                backRight.setVelocity(curVel);
                dtime = runtime.seconds() - time;
                time = runtime.seconds();
            } else if ((Math.abs(fL_TRGT - fLPos) < DIST) && (Math.abs(fL_TRGT - fLPos) < dPos) && (Math.abs(curVel) > 0)) {
                curVel = Math.signum(curVel)*Math.max(Math.abs(curVel) - MIN_VEL_STEP,0);
                if (curVel == 0) {
                    dPos = -1;
                    curVel = Math.signum(fL_TRGT - fLPos)*MIN_VEL_STEP;
                }
                else {
                    dPos = (int) (Math.pow(curVel,2)/(2*MAX_ANGACC));
                }
                frontLeft.setVelocity(-curVel);
                frontRight.setVelocity(curVel);
                backLeft.setVelocity(-curVel);
                backRight.setVelocity(curVel);
                dtime = - runtime.seconds() + time;
                time = runtime.seconds();
            }
            fLPos = frontLeft.getCurrentPosition();

            if (dtime != 0) {
                minAcc = Math.min(minAcc,curAcc);
                maxAcc = Math.max(maxAcc,curAcc);
                curAcc = MIN_VEL_STEP/dtime;
                avgAcc = ((avgAcc*n)+curAcc)/++n;
            }

            maxVel = Math.max(maxVel,curVel);

            telemetry.addData("Current Position: ", fLPos);
            telemetry.addData("Current Velocity: ", curVel);
            telemetry.addData("Current Acceleration: ", curAcc);
            telemetry.update();
        }

        frontLeft.setVelocity(0);
        frontRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);

        sleep(125);

        telemetry.addData("MIN_DIST: ", MIN_DIST / DEGREES_TO_TICKS);
        telemetry.addData("Tolerance: ", frontLeft.getTargetPositionTolerance() / DEGREES_TO_TICKS);
        telemetry.addData("Within Tolerance.", "");
        telemetry.addData("Maximum Velocity: ", maxVel);
        telemetry.addData("Minimum Acceleration: ", minAcc);
        telemetry.addData("Average Acceleration: ", avgAcc);
        telemetry.addData("Maximum Acceleration: ", maxAcc);
        telemetry.update();
    }

    public void moveAngTest(double degrees) {
        double MIN_VEL_STEP = MAX_ANGVEL/100;
        double MIN_ANG = Math.pow(MAX_ANGVEL,2)/(2*MAX_ANGACC);
        double ANG_HALF = Math.min(MIN_ANG, Math.abs(degrees)/2);
        double ANG_INIT = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double ANG_TRGT = ANG_INIT + degrees;

        double curVel = Math.signum(degrees) * MIN_VEL_STEP;
        double angInt=ANG_INIT, angNow = ANG_INIT, angOld = ANG_INIT, angDel=0;
        double dAng = Math.pow(curVel,2)/(2*MAX_ANGACC);

        double time = runtime.seconds(), dtime=0, curAcc=0, avgAcc=0, minAcc=MAX_LINACC, maxAcc=0, maxVel=0;
        int n=0;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setVelocity(-curVel*DEGREES_TO_TICKS);
        frontRight.setVelocity(curVel*DEGREES_TO_TICKS);
        backLeft.setVelocity(-curVel*DEGREES_TO_TICKS);
        backRight.setVelocity(curVel*DEGREES_TO_TICKS);

        while (Math.abs(ANG_TRGT - angInt) > 2) {
            if ((Math.abs(angInt - ANG_INIT) < ANG_HALF) && (Math.abs(angInt - ANG_INIT) > dAng) && (Math.abs(curVel) < MAX_ANGVEL-0.1*MIN_VEL_STEP)) {
                curVel = Math.signum(curVel)*Math.min(Math.abs(curVel) + MIN_VEL_STEP,MAX_ANGVEL);
                if (Math.abs(curVel) < MAX_ANGVEL) dAng = Math.pow(curVel,2)/(2*MAX_ANGACC);
                else dAng = Math.pow(MAX_ANGVEL-MIN_VEL_STEP,2)/(2*MAX_ANGACC);
                frontLeft.setVelocity(-curVel*DEGREES_TO_TICKS);
                frontRight.setVelocity(curVel*DEGREES_TO_TICKS);
                backLeft.setVelocity(-curVel*DEGREES_TO_TICKS);
                backRight.setVelocity(curVel*DEGREES_TO_TICKS);
                dtime = runtime.seconds() - time;
                time = runtime.seconds();
                telemetry.addData("Speeding Up", "");
            } else if ((Math.abs(ANG_TRGT - angInt) < ANG_HALF) && (Math.abs(ANG_TRGT - angInt) < dAng) && (Math.abs(curVel) > 0.1*MIN_VEL_STEP)) {
                curVel = Math.signum(curVel)*Math.max(Math.abs(curVel) - MIN_VEL_STEP,0);
                if (curVel == 0) {
                    dAng = -1;
                    curVel = Math.signum(ANG_TRGT - angInt)*MIN_VEL_STEP;
                }
                else {
                    dAng = Math.pow(curVel,2)/(2*MAX_ANGACC);
                }
                frontLeft.setVelocity(-curVel*DEGREES_TO_TICKS);
                frontRight.setVelocity(curVel*DEGREES_TO_TICKS);
                backLeft.setVelocity(-curVel*DEGREES_TO_TICKS);
                backRight.setVelocity(curVel*DEGREES_TO_TICKS);
                dtime = - runtime.seconds() + time;
                time = runtime.seconds();
                telemetry.addData("Slowing Down", "");
            }
            angNow = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            angDel = angNow - angOld;
            angDel += ((angDel < -180) ? 360 : (angDel >= 180) ? -360 : 0);
            angInt += angDel;
            angOld = angNow;

            if (dtime != 0) {
                curAcc = MIN_VEL_STEP/dtime;
                avgAcc = ((avgAcc*n)+curAcc)/++n;
                minAcc = Math.min(minAcc,curAcc);
                maxAcc = Math.max(maxAcc,curAcc);
            }

            maxVel = Math.max(maxVel,curVel);

            telemetry.addData("Current Position: ", angNow);
            telemetry.addData("Current Displacement: ", angDel);
            telemetry.addData("Current Integrated Position: ", angInt);
            telemetry.addData("Current Integrated Displacement: ", angInt - ANG_INIT);
            telemetry.addData("Target Position: ",ANG_TRGT);
            //telemetry.addData("Current Velocity: ", curVel);
            //telemetry.addData("Current Velocity (Ticks): ", curVel*DEGREES_TO_TICKS);
            //telemetry.addData("Current Acceleration: ", curAcc);
            telemetry.update();
        }

        frontLeft.setVelocity(0);
        frontRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);

        sleep(125);

        telemetry.addData("MIN_ANG: ", MIN_ANG);
        telemetry.addData("Maximum Velocity: ", maxVel);
        telemetry.addData("Minimum Acceleration: ", minAcc);
        telemetry.addData("Average Acceleration: ", avgAcc);
        telemetry.addData("Maximum Acceleration: ", maxAcc);
        telemetry.update();
    }

    public void moveLat(double distance) {
        double MIN_VEL_STEP = MAX_LATVEL/100;
        int MIN_DIST = (int) (Math.pow(MAX_LATVEL,2)/(2*MAX_LATACC));
        int MOVE = (int) (Math.round(distance * LAT_BIAS * INCHES_TO_TICKS));
        int DIST = Math.min(MIN_DIST,Math.abs(MOVE)/2);
        int fL_INIT = frontLeft.getCurrentPosition();
        int fL_TRGT = fL_INIT + MOVE;

        double curVel = Math.signum(distance) * MIN_VEL_STEP;
        int fLPos = fL_INIT;
        int dPos = (int) (Math.pow(curVel,2)/(2*MAX_LATACC));

        double time = runtime.seconds(), dtime=0, curAcc=0, avgAcc=0, minAcc=MAX_LATACC, maxAcc=0, maxVel=0;
        int n=0;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setVelocity(curVel);
        frontRight.setVelocity(-curVel);
        backLeft.setVelocity(-curVel);
        backRight.setVelocity(curVel);

        while (Math.abs(fL_TRGT - fLPos) > frontLeft.getTargetPositionTolerance()) {
            if ((Math.abs(fLPos - fL_INIT) < DIST) && (Math.abs(fLPos - fL_INIT) > dPos) && (Math.abs(curVel) < MAX_LATVEL)) {
                curVel = Math.signum(curVel)*Math.min(Math.abs(curVel) + MIN_VEL_STEP,MAX_LATVEL);
                if (Math.abs(curVel) < MAX_LATVEL) dPos = (int) (Math.pow(curVel,2)/(2*MAX_LATACC));
                else dPos = (int) (Math.pow(MAX_LATVEL-MIN_VEL_STEP,2)/(2*MAX_LATACC));
                frontLeft.setVelocity(curVel);
                frontRight.setVelocity(-curVel);
                backLeft.setVelocity(-curVel);
                backRight.setVelocity(curVel);
                dtime = runtime.seconds() - time;
                time = runtime.seconds();
            } else if ((Math.abs(fL_TRGT - fLPos) < DIST) && (Math.abs(fL_TRGT - fLPos) < dPos) && (Math.abs(curVel) > 0)) {
                curVel = Math.signum(curVel)*Math.max(Math.abs(curVel) - MIN_VEL_STEP,0);
                if (curVel == 0) {
                    dPos = -1;
                    curVel = Math.signum(fL_TRGT - fLPos)*MIN_VEL_STEP;
                }
                else {
                    dPos = (int) (Math.pow(curVel,2)/(2*MAX_LATACC));
                }
                frontLeft.setVelocity(curVel);
                frontRight.setVelocity(-curVel);
                backLeft.setVelocity(-curVel);
                backRight.setVelocity(curVel);
                dtime = - runtime.seconds() + time;
                time = runtime.seconds();
            }
            fLPos = frontLeft.getCurrentPosition();

            if (dtime != 0) {
                minAcc = Math.min(minAcc,curAcc);
                maxAcc = Math.max(maxAcc,curAcc);
                curAcc = MIN_VEL_STEP/dtime;
                avgAcc = ((avgAcc*n)+curAcc)/++n;
            }

            maxVel = Math.max(maxVel,curVel);

            telemetry.addData("Current Position: ", fLPos);
            telemetry.addData("Current Velocity: ", curVel);
            telemetry.addData("Current Acceleration: ", curAcc);
            telemetry.update();
        }

        frontLeft.setVelocity(0);
        frontRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);

        sleep(125);

        telemetry.addData("MIN_DIST: ", MIN_DIST / INCHES_TO_TICKS);
        telemetry.addData("Tolerance: ", frontLeft.getTargetPositionTolerance() / INCHES_TO_TICKS);
        telemetry.addData("Within Tolerance.", "");
        telemetry.addData("Maximum Velocity: ", maxVel);
        telemetry.addData("Minimum Acceleration: ", minAcc);
        telemetry.addData("Average Acceleration: ", avgAcc);
        telemetry.addData("Maximum Acceleration: ", maxAcc);
        telemetry.update();
    }

    public void placePixel() {
        double angFinal = 160.0;

        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armPosition=0;

        armLeft.setTargetPosition(armPosition);
        armRight.setTargetPosition(armPosition);
        armLeft.setPower(1);
        armRight.setPower(1);

        while (armPosition/ARM_DEGREES_TO_TICKS < angFinal) {
            armPosition++;
            armLeft.setTargetPosition(armPosition);
            armRight.setTargetPosition(armPosition);
            //sleep(5);
            if (armPosition/ARM_DEGREES_TO_TICKS >= BOARD_ANGLE)
                wristArm.setPosition( ((armPosition/ARM_DEGREES_TO_TICKS)-BOARD_ANGLE) * WRIST_DEGREES_TO_TICKS );
        }

        //sleep(1000);
        wristLeft.setPosition(0.8);
        wristRight.setPosition(0.5);
        sleep(1000);
        wristLeft.setPosition(0.45);
        wristRight.setPosition(0.85);
        //sleep(1000);

        boolean flip = true;
        while (armPosition/ARM_DEGREES_TO_TICKS > 10) {
            armPosition--;
            armLeft.setTargetPosition(armPosition);
            armRight.setTargetPosition(armPosition);
            sleep(1);
            if (armPosition/ARM_DEGREES_TO_TICKS < BOARD_ANGLE && flip) {
                wristArm.setPosition(1);
                flip = false;
            }
        }

        armLeft.setPower(0);
        armRight.setPower(0);
    }

    public void lowerArm() {

        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armPosition=100;

        armLeft.setTargetPosition(armPosition);
        armLeft.setPower(1);
        armRight.setTargetPosition(armPosition);
        armRight.setPower(1);

        while (armLeft.isBusy() || armRight.isBusy()) {};

        armLeft.setPower(1);
        armRight.setPower(1);

        while (armPosition>-130) {
            armLeft.setTargetPosition(--armPosition);
            armRight.setTargetPosition(--armPosition);
        }

        armLeft.setPower(0);
        armRight.setPower(0);
    }

    public void sleep(double duration) {  //duration in ms
        double start = runtime.milliseconds();
        while (runtime.milliseconds() < start + duration) {};
    }

}   // end class