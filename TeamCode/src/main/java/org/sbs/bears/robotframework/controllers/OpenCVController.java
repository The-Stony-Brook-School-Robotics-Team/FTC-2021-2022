package org.sbs.bears.robotframework.controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.archive.B;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;

public class OpenCVController {
    // MARK - Class Variables
    static final int STREAM_WIDTH = 1920;
    static final int STREAM_HEIGHT = 1080;
    DuckOpenCVEngineBlueSpline engine1;
    DuckOpenCVEngineBlueSimple engine2;
    DuckOpenCVEngineRedSpline engine3;
    DuckOpenCVEngineRedSimple engine4;
    OpenCvPipeline currentEngine;
    OpenCvCamera webcam;
    AutonomousMode mode;
    public OpenCVController(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode)
    {
        // Prepare the OpenCV Configuration and Engine
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "WebcamMain");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        switch(mode) {
            case RedSimple:
                engine4 = new DuckOpenCVEngineRedSimple();
                webcam.setPipeline(engine4);
                currentEngine = engine4;
            case RedSpline:
                engine3 = new DuckOpenCVEngineRedSpline();
                webcam.setPipeline(engine3);
                currentEngine = engine3;
            case BlueSimple:
                engine2 = new DuckOpenCVEngineBlueSimple();
                webcam.setPipeline(engine2);
                currentEngine = engine2;
            case BlueSpline:
                engine1 = new DuckOpenCVEngineBlueSpline();
                webcam.setPipeline(engine1);
                currentEngine = engine1;
        }
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
        switch(mode) {
            case RedSimple:
                return new int[]{engine4.getAYanalysis(), engine4.getBYanalysis(), engine4.getCYanalysis()};
            case RedSpline:
                return new int[]{engine3.getAYanalysis(), engine3.getBYanalysis(), engine3.getCYanalysis()};
            case BlueSimple:
                return new int[]{engine2.getAYanalysis(), engine2.getBYanalysis(), engine2.getCYanalysis()};
            case BlueSpline:
                return new int[]{engine1.getAYanalysis(), engine1.getBYanalysis(), engine1.getCYanalysis()};
        }
        return new int[]{-1,-1,-1};
    }
    public TowerHeightFromDuck getWhichTowerHeight() {
        /*

        BLUE SIDE

                                    _
                                  _| |_
         _      _      _        _|     |_         _      _      _
        [_]    [_]    [_]      [_________]       [_]    [_]    [_]
         A      B      C                          A      B      C



        A1
        B2
        C3

        */
        int[] result = getAnalysis();
        int Aresult = result[0];
        int Bresult = result[1];
        int Cresult = result[2];
        int bestResult = Math.max(Aresult,Math.max(Bresult,Cresult));
        DuckPosition pos = DuckPosition.NA;
        if(bestResult == Aresult)
        {
            pos = DuckPosition.A;
        }
        else if(bestResult == Bresult)
        {
            pos = DuckPosition.B;
        }
        else if(bestResult == Cresult)
        {
            pos = DuckPosition.C;
        }
        if (pos.equals(DuckPosition.NA)) {
            return TowerHeightFromDuck.NA; // ERROR
        }


        switch(pos) {
            case A:
                return TowerHeightFromDuck.ONE;
            case B:
                return TowerHeightFromDuck.TWO;
            case C:
                return TowerHeightFromDuck.THREE;
        }
        return TowerHeightFromDuck.NA;
    }
    public void shutDown() {
        webcam.stopStreaming();
        engine1 = null;
    }
    enum DuckPosition {
        A,
        B,
        C,
        NA
    }
}

