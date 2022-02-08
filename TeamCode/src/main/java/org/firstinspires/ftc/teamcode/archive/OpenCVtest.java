package org.firstinspires.ftc.teamcode.archive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
public class OpenCVtest extends OpMode {
    //Continuation<? extends Consumer<Bitmap>> continuation;

    static final int STREAM_WIDTH = 1920;
    static final int STREAM_HEIGHT = 1080;
    OpenCvWebcam webcam;
    FtcDashboard dash;
    //VuforiaLocalizer vuforia;
    //OpenCvCamera vuforiaPassthroughCam;
    OpenCVEngineCYMK pipeline;

    @Override
    public void init() {
        //continuation  = new Continuation<Consumer<Bitmap>>();
        //dash = FtcDashboard.getInstance();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       // int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);


        WebcamName webcamName = null;
            webcamName = hardwareMap.get(WebcamName.class, "WebcamMain");
           webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
           pipeline = new OpenCVEngineCYMK();
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

        telemetry.addData("Rect A: ",pipeline.getAanalysis());
        telemetry.addData("Rect B: ",pipeline.getBanalysis());
        telemetry.addData("Rect C: ",pipeline.getCanalysis());
        telemetry.addData("Rect ACr: ",pipeline.getACranalysis());
        telemetry.addData("Rect BCr: ",pipeline.getBCranalysis());
        telemetry.addData("Rect CCr: ",pipeline.getCCranalysis());
        telemetry.addData("Rect AY: ",pipeline.getAYanalysis());
        telemetry.addData("Rect BY: ",pipeline.getBYanalysis());
        telemetry.addData("Rect CY: ",pipeline.getCYanalysis());
        telemetry.addData("Rect Ay: ",pipeline.getAYanalysis());
        telemetry.addData("Rect By: ",pipeline.getBYanalysis());
        telemetry.addData("Rect Cy: ",pipeline.getCYanalysis());
        telemetry.update();

    }

    @Override
    public void loop() {
        telemetry.addData("","");
        telemetry.update();
        //webcam.getFrameBitmap();
        /*if(gamepad1.a){
        try {
            Mat processed = pipeline;
            Bitmap bmp = Bitmap.createBitmap(processed.width(),processed.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(processed,bmp);
            dash.sendImage(bmp); // send image to dashboard view
        } catch (InterruptedException e) {
            e.printStackTrace();
        }}*/


    }

}
