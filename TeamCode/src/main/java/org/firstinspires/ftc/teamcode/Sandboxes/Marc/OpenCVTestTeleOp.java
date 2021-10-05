package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.BearsUtil.OpenCVEngine;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class OpenCVTestTeleOp extends OpMode {
    static final int STREAM_WIDTH = 640;
    static final int STREAM_HEIGHT = 480;
    OpenCVEngine pipeline;
    OpenCvWebcam webcam;
    @Override
    public void init() {

        WebcamName webcamName = null;
            webcamName = hardwareMap.get(WebcamName.class, "WebcamMain");
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
            pipeline = new OpenCVEngine();
            webcam.setPipeline(pipeline);

            // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
            // out when the RC activity is in portrait. We do our actual image processing assuming
            // landscape orientation, though.
//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

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

    @Override
    public void loop() {
        telemetry.addData("Rect A: ",pipeline.getAanalysis());
        telemetry.addData("Rect B: ",pipeline.getBanalysis());
        telemetry.addData("Rect C: ",pipeline.getCanalysis());
        telemetry.update();
    }

    /*@Override
    public void stop() {
        webcam.stopStreaming();
    }*/
}
