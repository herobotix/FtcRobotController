package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

@Autonomous(name = "autoRedMain", group = "red")
public class autoRedMain extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotor armRight, armLeft, armMiddle;
    public Servo wristRight, wristLeft, wristArm, launcher;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    //private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_red.tflite";
    //private static final String[] LABELS = { "belement", };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    // Drive Ratios for FRONT/BACK LEFT/RIGHT motors.
    Integer cpr = 27;
    double gearRatio = 15.2;
    double diameter = 2.95276;
    double cpi = (cpr * gearRatio) / (Math.PI * diameter);
    double bias = 1;
    double driveConversion = (cpi * bias);

    IMU imu;

    String spikeMark;
    String[] location = {"InField","Blue"};
    boolean doPlacePixel = false;
    int armPosition;
    int yawAngle;
    double fun;

    @Override
    public void init() {
        initTfod();

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft"); //3
        frontRight = hardwareMap.get(DcMotor.class, "frontRight"); //1
        backLeft = hardwareMap.get(DcMotor.class, "backLeft"); //2
        backRight = hardwareMap.get(DcMotor.class, "backRight"); //0

        armRight = hardwareMap.get(DcMotor.class, "armRight");
        armLeft = hardwareMap.get(DcMotor.class, "armLeft");
        armMiddle = hardwareMap.get(DcMotor.class, "armMiddle");

        wristArm = hardwareMap.get(Servo.class, "wristArm");
        wristRight = hardwareMap.get(Servo.class, "wristRight");
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        launcher = hardwareMap.get(Servo.class, "launcher");

        imu = hardwareMap.get(IMU.class, "imu");    // Check hardwareMap on DriverHub, to determine whether ControlHub IMU or ExpansionHub IMU.

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        imuInit();
    }

    @Override
    public void init_loop() {
        if(gamepad1.a) location[0] = (Objects.equals(location[0],"InField")) ? "OutField" : "InField";
        if(gamepad1.b) location[1] = (Objects.equals(location[1],"Blue")) ? "Red" : "Blue";
        if(gamepad1.x) doPlacePixel = !doPlacePixel;

        armLeft.setTargetPosition(0);
        armRight.setTargetPosition(0);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Field Location:", location);
        telemetry.addData("Arm Position: ", armLeft.getCurrentPosition());
        telemetry.addData("Placing pixel: ", doPlacePixel);

        telemetryTfod();

        imuTelemetry();

        telemetry.update();
    }

    @Override
    public void loop() {}

    @Override
    public void start() {
        visionPortal.stopStreaming(); // stops the stream once we click start to save CPU and random detections.

        telemetry.addData("Arm Position: ", armLeft.getCurrentPosition());
        telemetry.addData("SpikeMark:", spikeMark);
        telemetry.update();

        double pwr=0.3;

        wristLeft.setPosition(0.45);
        wristRight.setPosition(0.85);

        if(Objects.equals(location, "left")) {
            if(Objects.equals(spikeMark, "left")) {
                strafeToPosition(0.3, 3);
                driveToPosition(0.3,20);
                turnLeftDegrees(0.3, 55);
                driveToPosition(0.3,5);
                driveToPosition(0.3,-6);
                turnLeftDegrees(0.3, 125);
                lowerArm();
                driveToPosition(0.3, -34);
                turnRightDegrees(0.3, 90.5);
                driveToPosition(0.4, -87);
                telemetry.addLine("1 - left");
                telemetry.update();
            } else if(Objects.equals(spikeMark, "middle")) {
                strafeToPosition(0.3, 3);
                driveToPosition(0.3, 29); //drive froward 29 inches
                driveToPosition(0.4, -9);
                lowerArm();
                strafeToPosition(0.2, -18);
                turnRightDegrees(0.3, 180);  // FOR REVERSE DRIVING MODE
                driveToPosition(0.3, -34);
                turnRightDegrees(0.3, 90);
                driveToPosition(0.2, -18);
                driveToPosition(0.4, -85);
                telemetry.addLine("2- left");
                telemetry.update();
            } else { //right
                strafeToPosition(0.2, 3);
                driveToPosition(0.3, 20);
                turnRightDegrees(0.1, 55);
                driveToPosition(0.3, 5);
                driveToPosition(0.3, -6);
                turnLeftDegrees(0.3, 234);
                lowerArm();
                driveToPosition(0.3, -32);
                turnRightDegrees(0.3, 89.5);
                driveToPosition(0.4, -86);
                telemetry.addLine("3 - left");
                telemetry.update();
            }
            if (doPlacePixel) {
                strafeToPosition(0.3, -26); //strafe right 24 inches
                placePixel();
            }
        } else {
            if (Objects.equals(spikeMark, "left")) {
                strafeToPosition(0.2, 3);
                driveToPosition(0.3, 20);
                turnLeftDegrees(0.1, 55);
                driveToPosition(0.3, 5);
                driveToPosition(0.3, -6);
                turnRightDegrees(0.3, 55);
                driveToPosition(0.3, -15);
                lowerArm();
                turnLeftDegrees(0.3,87);
                driveToPosition(0.3, -38);
                telemetry.addLine("1 - right");
                telemetry.update();
            } else if (Objects.equals(spikeMark, "middle")) {
                strafeToPosition(0.2, 3);
                driveToPosition(0.3, 29); //drive froward 10 inches
                driveToPosition(0.3, -24);
                lowerArm();
                turnLeftDegrees(0.3, 87);
                driveToPosition(0.3, -40);
                telemetry.addLine("2 - right");
                telemetry.update();
            } else { //right
                strafeToPosition(0.3, 12);
                driveToPosition(0.3,20);
                driveToPosition(0.3, -15);
                lowerArm();
                turnLeftDegrees(0.3,90);
                driveToPosition(0.3, -28);
                telemetry.addLine("3 - right");
                telemetry.update();
            }
            if (doPlacePixel) {
                strafeToPosition(0.3, 24);
                placePixel();
            }
        }

    }

    private void initTfod() {

        tfod = new TfodProcessor.Builder()
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)
                //   The following default settings are available to un-comment and edit as needed to
                //   set parameters for custom models.
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.75f);

        //visionPortal.setProcessorEnabled(tfod, true);

    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        double x=-1;

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f", x);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

        }

        if (currentRecognitions.size()==0) x = -1;

        if (x >= 175) spikeMark = "middle";
        else if (x >= 0) spikeMark = "left";
        else spikeMark = "right";
        telemetry.addData("SpikeMark:", spikeMark);
    }

    public void driveToPosition(double power, double inches) {

        int move = (int) (Math.round(inches * driveConversion));//

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {}

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void strafeToPosition(double power, double inches) {

        int move = (int)(Math.round(inches * driveConversion));

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {}

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnRightDegrees(double power, double degrees) {

        double dtoi = 23.50/90.0;
        int move = (int)(Math.round(degrees * dtoi * driveConversion));

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + move);
        backRight.setTargetPosition(backRight.getCurrentPosition() - move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) {
            telemetry.addData("Left", frontLeft.getCurrentPosition());
            telemetry.addData("Fun", fun);
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnLeftDegrees(double power, double degrees) {

        double dtoi = 23.50/90.0;
        int move = (int)(Math.round(degrees * dtoi * driveConversion));

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - move);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + move);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - move);
        backRight.setTargetPosition(backRight.getCurrentPosition() + move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Left", frontLeft.getCurrentPosition());
            telemetry.addData("Fun", fun);
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void sleep(double duration) {  //duration in ms
        double start = runtime.milliseconds();
        while (runtime.milliseconds() < start + duration) {};
    }

    public void placePixel() {
        armPosition=-130;

        armLeft.setTargetPosition(armPosition);
        armRight.setTargetPosition(armPosition);
        armLeft.setPower(1);
        armRight.setPower(1);

        while (armPosition<300) {
            armLeft.setTargetPosition(++armPosition);
            armRight.setTargetPosition(++armPosition);
        }

        sleep(1000);
        wristArm.setPosition(0);
        wristLeft.setPosition(0.8);
        wristRight.setPosition(0.5);
        sleep(1000);

        while (armPosition>100) {
            armLeft.setTargetPosition(--armPosition);
            armRight.setTargetPosition(--armPosition);
        }

        wristLeft.setPosition(0.45);
        wristRight.setPosition(0.85);
        wristArm.setPosition(1);
        sleep(1000);

        while (armPosition>-130) {
            armLeft.setTargetPosition(--armPosition);
            armRight.setTargetPosition(--armPosition);
        }

        armLeft.setPower(0);
        armRight.setPower(0);
    }

    public void lowerArm() {

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

    public void imuInit() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }

    public void imuTelemetry() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        yawAngle = (int) orientation.getYaw(AngleUnit.DEGREES);

        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
        telemetry.addData("YawAngle", yawAngle);
    }

    public void turnImu(int degrees) {
        //move right
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        degrees = (int)(degrees * 0.9);
        yawAngle = (int) orientation.getYaw(AngleUnit.DEGREES);

        int turn = yawAngle - degrees;

        if(degrees > 0) {

            frontLeft.setPower(0.2);
            backLeft.setPower(0.2);

            frontRight.setPower(-0.2);
            backRight.setPower(-0.2);

            while(!(yawAngle >= degrees)) {
                yawAngle = (int) orientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Yaw Value", yawAngle);
                telemetry.update();
            }

            frontLeft.setPower(0);
            backLeft.setPower(0);

            frontRight.setPower(0);
            backRight.setPower(0);

        } else {
            //move left
            frontLeft.setPower(-0.2);
            backLeft.setPower(-0.2);

            frontRight.setPower(0.2);
            backRight.setPower(0.2);

            while(!(yawAngle <= degrees)) {
                yawAngle = (int) orientation.getYaw(AngleUnit.DEGREES);
                telemetry.addData("Yaw Value", yawAngle);
                telemetry.update();
            }

            frontLeft.setPower(0);
            backLeft.setPower(0);

            frontRight.setPower(0);
            backRight.setPower(0);

        }

    }

}   // end class
