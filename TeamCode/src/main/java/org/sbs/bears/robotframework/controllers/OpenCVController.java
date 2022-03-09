package org.sbs.bears.robotframework.controllers;

import static java.lang.Thread.sleep;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.doclint.Checker;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.enums.DuckPosition;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;

import java.util.concurrent.atomic.AtomicReference;

public class OpenCVController {
    // MARK - Class Variables
    static final int STREAM_WIDTH = 1920;
    static final int STREAM_HEIGHT = 1080;
    DuckOpenCVEngine engine;

    public static volatile boolean doAnalysisMaster = true;


    public static boolean isDuck = false; // false by default: TSE

    OpenCvPipeline currentEngine;
    OpenCvCamera webcam;
    AutonomousMode mode;
    public OpenCVController(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode)
    {
        // Prepare the OpenCV Configuration and Engine
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "WebcamMain");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        Log.d("OpenCVController","Init Engine");
        // TODO switch for all autonomous types only on constructor
        if(isDuck) {
            engine = new DuckOpenCVEngineBlueFull();
        }
        else {
            switch(mode) {
                case BlueStatesDuckSimple:
                    engine = new CapstoneOpenCVEngineBlueSimple();
                    break;
                case BlueStatesWarehouse:
                    engine = new CapstoneOpenCVEngineBlueFull();
                    break;
                default:
                    engine = new CapstoneOpenCVEngineRedFull();
            }
        }
        webcam.setPipeline(engine);
        currentEngine = engine;
        Log.d("OpenCVController","Init Complete");
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
        /*switch(mode) {
            case RedSimple:
                return new int[]{engine4.getAYanalysis(), engine4.getBYanalysis(), engine4.getCYanalysis()};
            case RedSpline:
                return new int[]{engine3.getAYanalysis(), engine3.getBYanalysis(), engine3.getCYanalysis()};
            case BlueSimple:
                return new int[]{engine2.getAYanalysis(), engine2.getBYanalysis(), engine2.getCYanalysis()};
            case BlueSpline:
                return new int[]{engine1.getAYanalysis(), engine1.getBYanalysis(), engine1.getCYanalysis()};
        }*/
        return new int[]{engine.getAYanalysis(), engine.getBYanalysis(), engine.getCYanalysis()};
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
        while(doAnalysisMaster) {
            try {
                sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        int[] result = getAnalysis();

        int Aresult = result[0];
        int Bresult = result[1];
        int Cresult = result[2];
        int bestResult = Math.max(Aresult,Math.max(Bresult,Cresult));
        System.out.println();
        System.out.println(result[0]);
        System.out.println(result[1]);
        System.out.println(result[2]);
        System.out.println(bestResult);

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
        engine = null;

    }

    public boolean prepareWhiteLineEngine() {
        engine = new WhiteLineAvailOpenCVEngineBlueSimple();
        webcam.setPipeline(engine);
        currentEngine = engine;
        Log.d("OpenCVController","Init Complete");
        AtomicReference<Boolean> FLAG = new AtomicReference<>();
        FLAG.set(true);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                Log.d("OpenCVController","Error preparing white line engine. Aborting...");
                FLAG.set(false);
            }
        });
        return FLAG.get();
    }

    public boolean getWhiteLineAvailable() {
        return engine.getPosition().equals(DuckPosition.A);
        // TODO finish this
    }
}

