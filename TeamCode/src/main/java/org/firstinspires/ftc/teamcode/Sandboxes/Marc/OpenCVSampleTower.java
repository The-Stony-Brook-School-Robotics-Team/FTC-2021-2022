package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.sbs.bears.robotframework.controllers.DuckOpenCVEngineBlueFull;
import org.sbs.bears.robotframework.controllers.TowerOpenCVEngine;


@Autonomous(name="A - OpenCV Tower Analysis")
public class OpenCVSampleTower extends OpMode {

    // MARK - Class Variables
    static final int STREAM_WIDTH = 320;
    static final int STREAM_HEIGHT = 240;
    TowerOpenCVEngine engine;
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
        engine = new TowerOpenCVEngine();
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


    }
    // MARK - Stop of OpMode Loop
}
