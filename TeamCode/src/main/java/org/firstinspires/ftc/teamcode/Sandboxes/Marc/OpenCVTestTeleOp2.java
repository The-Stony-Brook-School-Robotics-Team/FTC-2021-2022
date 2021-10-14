package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import static java.lang.Thread.sleep;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.BearsUtil.BasicOpenCVEngine;
import org.firstinspires.ftc.teamcode.BearsUtil.OpenCVEngine;
import org.firstinspires.ftc.teamcode.BearsUtil.OpenCVEngineCYMK;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class OpenCVTestTeleOp2 extends OpMode {
    //Continuation<? extends Consumer<Bitmap>> continuation;

    static final int STREAM_WIDTH = 1920;
    static final int STREAM_HEIGHT = 1080;
    OpenCvWebcam webcam;
    FtcDashboard dash;
    //VuforiaLocalizer vuforia;
    //OpenCvCamera vuforiaPassthroughCam;
    BasicOpenCVEngine pipeline;
    boolean flag = false;
    boolean secondFlag = false;

    @Override
    public void init() {
        //continuation  = new Continuation<Consumer<Bitmap>>();
        //dash = FtcDashboard.getInstance();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);


        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "WebcamMain");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new BasicOpenCVEngine();
        webcam.setPipeline(pipeline);
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
        telemetry.addData("Rect AY: ",pipeline.getAYanalysis());
        telemetry.addData("Rect BY: ",pipeline.getBYanalysis());
        telemetry.addData("Rect CY: ",pipeline.getCYanalysis());

        //telemetry.addData("Rect Ay: ",pipeline.getAYanalysis());

        telemetry.update();
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);





    }

    @Override
    public void start() {
        super.start();
        BasicOpenCVEngine.doAnalysis = true;
        webcam.pauseViewport();
    }

    @Override
    public void loop() {


       // telemetry.addData("Rect A: ",pipeline.getAanalysis());

        //telemetry.addData("Rect ACr: ",pipeline.getACranalysis());
        if(!flag) {
        int A = pipeline.getAYanalysis();
        if(A != 0) {flag = true;}
        telemetry.addData("Rect AY: ",pipeline.getAYanalysis());
        telemetry.addData("Rect BY: ",pipeline.getBYanalysis());
        telemetry.addData("Rect CY: ",pipeline.getCYanalysis());
        telemetry.addData("Position: ", pipeline.getPosition());
        //telemetry.addData("Rect Ay: ",pipeline.getAYanalysis());

        telemetry.update();
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        }
        else if(!secondFlag) {
            BasicOpenCVEngine.doAnalysis = false;
            webcam.pauseViewport();
            webcam.stopStreaming();
            secondFlag = true;
        }

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
