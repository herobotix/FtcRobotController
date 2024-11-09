/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//import com.qualcomm.robotcore.util.ElapsedTime;

//import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Blue_Copy", group = "Concept")

public class Blue_Copy extends LinearOpMode {

    //private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    //private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/BlueDroid.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    //private static final String[] LABELS = {
    //   "blueDroid",
    //};

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    //private VisionPortal visionPortal;
    
    //private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    //private Servo rotateClaw;
    //private Servo claw;

    //double x;
    //double y;
    //int sector;
    //double object;
	//int step;
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        //rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");
        //claw = hardwareMap.get(Servo.class, "clawServo");
        
		FrontLeft.setDirection(DcMotor.Direction.FORWARD);
		FrontRight.setDirection(DcMotor.Direction.REVERSE);
		BackLeft.setDirection(DcMotor.Direction.FORWARD);
		BackRight.setDirection(DcMotor.Direction.REVERSE);
		
        //claw.setPosition(0.55);
        
        //FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
		/*
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		*/

        //initTfod();

        // Wait for the DS start button to be touched.
        //telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        //telemetry.addData(">", "Touch Play to start OpMode");
        //telemetry.update();
        
        waitForStart();
        //runtime.reset();
        
		//y = -1;
		//step = 0;
		
        //if (opModeIsActive()) {
			//rotateClaw.setPosition(.53);
			
			FrontLeft.setPower(0.1);
			FrontRight.setPower(0.1);
			BackLeft.setPower(0.1);
			BackRight.setPower(0.1);
			
			sleep(10000);
			
			FrontLeft.setPower(0.0);
			FrontRight.setPower(0.0);
			BackLeft.setPower(0.0);
			BackRight.setPower(0.0);
						
			/*
			
            while (opModeIsActive()) {
				
				//FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				//FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				//BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				//BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
				
				//forward(50);
				//sleep(10000);
                
				if (y>=0) {
					telemetryTfod();
					telemetry.update();
				} else {
					visionPortal.stopStreaming();
					if (y>200) {
						sector = 1;
					} else if (y<=200 && y>10) {
						sector = 2;
					} else {
						sector = 3;
					}
					telemetry.addData("spike", sector);
					telemetry.addData("y = ", y);
					telemetry.update();
					
					step = 1;					
				}
				
				switch (step) {
					case 1:
						forward(45+(10*sector));
						step = 2;
						sleep(5000);
						break;
					case 2:
						rotate(-15+(10*sector));
						if (sector==3) step = 3;
						else step = 4;
						sleep(3000);
						break;
					case 3:
						forward(18);
						step = 4;
						sleep(4000);
					case 4:
						claw.setPosition(0.25);
						step = 0;
					default:
						sleep(20);
				}
            }
			*/
        //}

        // Save more CPU resources when camera is no longer needed.
        //visionPortal.close();

    }   // end runOpMode()

	/*
    private void rotate(int cm){
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
    
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int distance;
    
        distance = (int) Math.round(112*cm/(7.5*Math.PI));

        FrontLeft.setTargetPosition(distance);
        FrontRight.setTargetPosition(distance);
        BackLeft.setTargetPosition(distance);
        BackRight.setTargetPosition(distance);
    
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

	private void move(int distance, int radius) {
		FrontLeft.setDirection(DcMotor.Direction.FORWARD);
		FrontRight.setDirection(DcMotor.Direction.REVERSE);
		BackLeft.setDirection(DcMotor.Direction.FORWARD);
		BackRight.setDirection(DcMotor.Direction.REVERSE);
		
		FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		//int distance;		
		//distance = (int) Math.round(112*cm/(7.5*Math.PI));
		
		
	}
    
    private void forward(int cm) {
		FrontLeft.setDirection(DcMotor.Direction.FORWARD);
		FrontRight.setDirection(DcMotor.Direction.REVERSE);
		BackLeft.setDirection(DcMotor.Direction.FORWARD);
		BackRight.setDirection(DcMotor.Direction.REVERSE);
		
		FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		int distance;
		
		distance = (int) Math.round(112*cm/(7.5*Math.PI));
		
		telemetry.addData("rotations", distance);
		telemetry.update();
		
		FrontLeft.setTargetPosition(distance);
		FrontRight.setTargetPosition(distance);
		BackLeft.setTargetPosition(distance);
		BackRight.setTargetPosition(distance);
		
		FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}
	*/
		
    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    
	/*
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

            // With the following lines commented out, the default TfodProcessor Builder
            // will load the default model for the season. To define a custom model to load, 
            // choose one of the following:
            //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
            //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            .setModelFileName(TFOD_MODEL_FILE)

            // The following default settings are available to un-comment and edit as needed to 
            // set parameters for custom models.
            .setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

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

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()
	*/

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
	
	/*
    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        object = currentRecognitions.size();
        
        //if (start == false) {// Step through the list of recognitions and display info for each one.
			for (Recognition recognition : currentRecognitions) {
				x = (recognition.getLeft() + recognition.getRight()) / 2 ;
				y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
			
				telemetry.addData(""," ");
				telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
				telemetry.addData("- Position", "%.0f / %.0f", x, y);
				telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
			}   // end for() loop
        //} //apart of if

    }   // end method telemetryTfod()
	*/

}   // end class
