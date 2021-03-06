package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.sbs.bears.robotframework.controllers.CapstoneOpenCVEngineBlueFull;
import org.sbs.bears.robotframework.controllers.DuckOpenCVEngineBlueFull;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.sbs.bears.robotframework.enums.DuckPosition;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;


/**
 * This OpMode is a sample OpenCV OpMode to analyze the position of the TSE on the Barcode.
 * @author Marc N.
 * @version 5.1
 */
//@Autonomous(name="OpenCV TSE Recog Test")
public class OpenCVSample extends OpMode {

    // MARK - Class Variables
    static final int STREAM_WIDTH = 1920;
    static final int STREAM_HEIGHT = 1080;
    CapstoneOpenCVEngineBlueFull engine;

    boolean flag = false; // flags for ensuring the time spent analyzing the frames.
    boolean secondFlag = false;
    OpenCvCamera webcam;


    // MARK - Initialization
    @Override
    public void init() {
        // Prepare the OpenCV Configuration and Engine
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "WebcamMain");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        engine = new CapstoneOpenCVEngineBlueFull();
        webcam.setPipeline(engine);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        });
        // Print analysis
        DuckPosition pos = engine.getPosition();
        TowerHeightFromDuck height = TowerHeightFromDuck.NA;
        switch(pos) {
            case A:
                height = TowerHeightFromDuck.ONE;
                break;
            case B:
                height =  TowerHeightFromDuck.TWO;
                break;
            case C:
                height =  TowerHeightFromDuck.THREE;
                break;
        }
        telemetry.addData("Position: ",pos);
        telemetry.addData("Height: ", height);

        telemetry.update();
    }
    // MARK - End of Initialization

    // MARK - Start of OpMode Run
    @Override
    public void start() {
        super.start();
        // Start the analysis
        //webcam.pauseViewport();
    }
    // MARK - End of OpMode Run

    // MARK - Start of OpMode Loop
    @Override
    public void loop() {
        telemetry.addData("Rect AY: ", engine.getAYanalysis());
        telemetry.addData("Rect BY: ", engine.getBYanalysis());
        telemetry.addData("Rect CY: ", engine.getCYanalysis());
        telemetry.addData("Square: ", engine.getPosition());
        telemetry.update();
        try {
            sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        /*if(!flag) {
            int A = engine.getAYanalysis();
            if(A != 0) {
                flag = true; // analyzed, so don't analyze anymore
            }
            else {
                return; // A == 0 : true Thus we save time by looping again.
            }
            // Analyzed: print the results
            telemetry.addData("Rect AY: ", engine.getAYanalysis());
            telemetry.addData("Rect BY: ", engine.getBYanalysis());
            telemetry.addData("Rect CY: ", engine.getCYanalysis());
            telemetry.addData("Position: ", engine.getPosition());
            telemetry.update();
            try {
                sleep(1000); // Sleep to give OpenCV time to shut down
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        else if(!secondFlag) {
            DuckOpenCVEngineBlueSpline.doAnalysis = false; // stop
            webcam.pauseViewport(); // shut down camera
            webcam.stopStreaming();  // shut down camera
            secondFlag = true;
        }*/

    }
    // MARK - Stop of OpMode Loop
}
