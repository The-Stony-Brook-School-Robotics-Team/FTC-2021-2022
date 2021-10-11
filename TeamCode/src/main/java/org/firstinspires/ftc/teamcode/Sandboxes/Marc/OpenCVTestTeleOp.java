package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.BearsUtil.OpenCVEngine;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class OpenCVTestTeleOp extends OpMode {
    static final int STREAM_WIDTH = 640;
    static final int STREAM_HEIGHT = 480;
    OpenCvWebcam webcam;
    FtcDashboard dash;
    VuforiaLocalizer vuforia;
    OpenCvCamera vuforiaPassthroughCam;
    OpenCVEngine pipeline;

    static int counter = 0;
    private boolean qA = false;


    @Override
    public void init() {
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        dash = FtcDashboard.getInstance();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        /*
         * Setup Vuforia
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);
        parameters.vuforiaLicenseKey = "AchLY5n/////AAABmfPrXXNeQExEn0GWbSMuuDgcuqqqRahuLeg8ae9KGJCC2bE6opIDhZYWxoS63VJHlBj0GAybNnhin4XwkqYj8K8IfvNyi/XGi4nsuAAN9xiTu5op3Oo3Mn26rX6HK+Nt8b1twME24e4irMWLo0siT9Hiw5Gp4LRPDTLsTZeHoD/KhLIUVw+ECSrf/VZBD13QoCutaWNL8P2aXseGIHgvU8C09GO7ri4rNMEpib9g6uhI3Co001e96UEEtvcJdliPltCUnjCIVHIHatrdalPWAReu6URLoEeNOPNCu27oLaDAbcgdZnoYsNwxryhRFSIJnLKOliWaYh2Ng2StCWFu48MA0nwmXQUG0EfYbXExtJYB";
        parameters.cameraDirection   = VuforiaLocalizer.CameraDirection.BACK;
        // Uncomment this line below to use a webcam
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforiaPassthroughCam = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters, viewportContainerIds[1]);
        pipeline = new OpenCVEngine();
        // Create a Vuforia passthrough "virtual camera"vuforiaPassthroughCam = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters, viewportContainerIds[1]);
        vuforiaPassthroughCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Using GPU acceleration can be particularly helpful when using Vuforia passthrough
                // mode, because Vuforia often chooses high resolutions (such as 720p) which can be
                // very CPU-taxing to rotate in software. GPU acceleration has been observed to cause
                // issues on some devices, though, so if you experience issues you may wish to disable it.
                vuforiaPassthroughCam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                vuforiaPassthroughCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
                vuforiaPassthroughCam.setPipeline(pipeline);

                // We don't get to choose resolution, unfortunately. The width and height parameters
                // are entirely ignored when using Vuforia passthrough mode. However, they are left
                // in the method signature to provide interface compatibility with the other types
                // of cameras.
                vuforiaPassthroughCam.startStreaming(0,0, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        dash.startCameraStream(vuforia,1);
        vuforia.setFrameQueueCapacity(0);


       // WebcamName webcamName = null;
            //webcamName = hardwareMap.get(WebcamName.class, "WebcamMain");
            //int cameraMonitorViewId = 1;// hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
          //  webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
          // pipeline = new OpenCVEngine();
          //  webcam.setPipeline(pipeline);

            // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
            // out when the RC activity is in portrait. We do our actual image processing assuming
            // landscape orientation, though.
//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

          /*  webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
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
            });*/
        vuforia.enableConvertFrameToBitmap();

    }

    @Override
    public void loop() {
        if(gamepad1.a  && !qA) {
            qA = true;
            //counter++;
            int[] AYCrCbAnalysis = new int[]{pipeline.getAYanalysis(), pipeline.getACranalysis(), pipeline.getAanalysis()};
            int[] BYCrCbAnalysis = new int[]{pipeline.getBYanalysis(), pipeline.getBCranalysis(), pipeline.getBanalysis()};
            int[] CYCrCbAnalysis = new int[]{pipeline.getCYanalysis(), pipeline.getCCranalysis(), pipeline.getCanalysis()};

            //int[] ACYMKAnalysis = pipeline.getACYMKanalysis();
           // int[] BCYMKAnalysis = pipeline.getBCYMKanalysis();
            //int[] CCYMKAnalysis = pipeline.getCCYMKanalysis();

            telemetry.addData("Rect A YCrCb: ", AYCrCbAnalysis);
            telemetry.addData("Rect B YCrCb: ", BYCrCbAnalysis);
            telemetry.addData("Rect C YCrCb: ", CYCrCbAnalysis);
            //telemetry.addData("Rect A CYMK: ", ACYMKAnalysis);
           // telemetry.addData("Rect B CYMK: ", BCYMKAnalysis);
            //telemetry.addData("Rect C CYMK: ", CCYMKAnalysis);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Rect A YCrCb: ", AYCrCbAnalysis);
            packet.put("Rect B YCrCb: ", BYCrCbAnalysis);
            packet.put("Rect C YCrCb: ", CYCrCbAnalysis);
          //  packet.put("Rect A CYMK: ", ACYMKAnalysis);
           // packet.put("Rect B CYMK: ", BCYMKAnalysis);
           // packet.put("Rect C CYMK: ", CCYMKAnalysis);


             // only one in 20 frames will be sent, may want to make this bigger or disappear altogether.
                try {
                    Mat processed = pipeline.processFrame(getMatVuforia());
                    Bitmap bmp = Bitmap.createBitmap(processed.width(), processed.height(), Bitmap.Config.RGB_565);
                    Utils.matToBitmap(processed, bmp);
                    dash.sendImage(bmp); // send image to dashboard view
                } catch (InterruptedException e) {
                    e.printStackTrace();
                    packet.put("Camera Status: ", "image failed to send");
                }

            dash.sendTelemetryPacket(packet);
        }
        if(qA && !gamepad1.a) {qA = false;}

    }
    public Mat getMatVuforia() throws InterruptedException {
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
        long numImages = frame.getNumImages();
        Image rgb = null;
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }

        /*rgb is now the Image object that weve used in the video*/
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

//put the image into a MAT for OpenCV
        Mat tmp = new Mat(rgb.getWidth(), rgb.getHeight(), CvType.CV_8UC4);
        Utils.bitmapToMat(bm, tmp);

//close the frame, prevents memory leaks and crashing
        frame.close();
        return tmp;
    }

    /*@Override
    public void stop() {
        webcam.stopStreaming();
    }*/
}
