import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "AutonomousParking", group = "Concept")
//@Disabled
public class AutonomousParking extends LinearOpMode
{

    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/BlueDroid.tflite";
    private static final String[] LABELS = {"blueDroid"};
    
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    
    private ElapsedTime runtime = new ElapsedTime();
    
    private DcMotorEx FrontLeft;
    private DcMotorEx FrontRight;
    private DcMotorEx BackLeft;
    private DcMotorEx BackRight;
    private Servo rotateClaw;
    private Servo claw;
    
    double x;
    double y;
    double object;
    int sector;
    boolean active;
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");
        claw = hardwareMap.get(Servo.class, "clawServo");

        claw.setPosition(0.55);
        
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        initTfod();
        
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        
        waitForStart();
        runtime.reset();
        
         if (opModeIsActive()) {
            rotateClaw.setPosition(.53);
            y=-1;
            
            active=true;
            while(active) {
                telemetryTfod();
                telemetry.update();
                if (y>=0 || runtime.seconds()>=10.0) {
                    visionPortal.close();
                    if (y>200) {
                        move(55.0,-20.0,5.0);  //Sector 1.
                        sector = 1;
                    } else if (y<=200 && y>10) {
                        move(65.0,+35.0,5.0);  //Sector 2.
                        sector = 2;
                    } else {
                        move(65.0,+80.0,5.0);  //Sector 3.
                        sector = 3;
                    }
                    while (FrontLeft.isBusy()) {
                        telemetry.addData("spike", sector);
                        telemetry.addData("y = ", y);
                        telemetry.update();
                        sleep(50);
                    }
                    claw.setPosition(0.25);
                    sleep(100);
                    active=false;
                }

            }
            //move(55.0,-20.0,5.0);  //Sector 1.
            //move(65.0,+35.0,5.0);  //Sector 2.
            //move(65.0,+80.0,5.0);  //Sector 3.
        }
        
        //FrontLeft.setVelocity(0.0);
        //FrontRight.setVelocity(0.0);
        //BackLeft.setVelocity(0.0);
        //BackRight.setVelocity(0.0);
    }
    
    private void move(double distance, double angle, double time) {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double conv, distL, distR, velL, velR;
        conv = 112/(7.5*Math.PI);  // ticks per centimeter
        distL = conv * (distance + 10.0*angle*(Math.PI/180.0));  // Distance change with angle change
        distR = conv * (distance - 10.0*angle*(Math.PI/180.0));  // +/- angle turns right/left
        velL = distL/time;
        velR = distR/time;
        
        //telemetry.addData("distL theory:", distL);
        //telemetry.addData("distL actual:", 0.5*(FrontLeft.getCurrentPosition()+BackLeft.getCurrentPosition()));
        //telemetry.addData("distR theory", distR);
        //telemetry.addData("distR actual:", 0.5*(FrontRight.getCurrentPosition()+BackRight.getCurrentPosition()));
        //telemetry.addData("velL theory", velL);
        //telemetry.addData("velL actual", 0.5*(FrontLeft.getVelocity()+BackLeft.getVelocity()));
        //telemetry.addData("velR theory", velR);
        //telemetry.addData("velR actual", 0.5*(FrontRight.getVelocity()+BackRight.getVelocity()));
        
        FrontLeft.setTargetPosition((int) distL);
        FrontRight.setTargetPosition((int) distR);
        BackLeft.setTargetPosition((int) distL);
        BackRight.setTargetPosition((int) distR);
        
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontLeft.setVelocity(velL);
        FrontRight.setVelocity(velR);
        BackLeft.setVelocity(velL);
        BackRight.setVelocity(velR);
    }
    
    private void initTfod() {
        tfod = new TfodProcessor.Builder()
            //.setModelAssetName(TFOD_MODEL_ASSET)
            .setModelFileName(TFOD_MODEL_FILE)
            .setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)
            .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        else builder.setCamera(BuiltinCameraDirection.BACK);
        //builder.setCameraResolution(new Size(640, 480));
        //builder.enableLiveView(true);
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        //builder.setAutoStopLiveView(false);
        builder.addProcessor(tfod);
        visionPortal = builder.build();
        //tfod.setMinResultConfidence(0.75f);
        //visionPortal.setProcessorEnabled(tfod, true);
    }

    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        object = currentRecognitions.size();
        
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
        
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }

}
