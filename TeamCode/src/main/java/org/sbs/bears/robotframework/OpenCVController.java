package org.sbs.bears.robotframework;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class OpenCVController {
    // MARK - Class Variables
    static final int STREAM_WIDTH = 1920;
    static final int STREAM_HEIGHT = 1080;
    DuckOpenCVEngine engine;
    OpenCvCamera webcam;
    public OpenCVController(HardwareMap hardwareMap, Telemetry telemetry)
    {
        // Prepare the OpenCV Configuration and Engine
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "WebcamMain");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        engine = new DuckOpenCVEngine();
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
    }
    public int[] getAnalysis() {
        return new int[]{engine.getAYanalysis(),engine.getBYanalysis(),engine.getCYanalysis()};
    }
    public void shutDown() {
        webcam.stopStreaming();
        engine = null;
    }
}
