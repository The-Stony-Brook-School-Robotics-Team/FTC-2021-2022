package org.firstinspires.ftc.teamcode.common.official.teleop.v1;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.official.teleop.v1.enums.ControllerModes;
import org.firstinspires.ftc.teamcode.common.official.teleop.v1.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.common.official.teleop.v1.runtime.ButtonHandler;
import org.firstinspires.ftc.teamcode.common.official.teleop.v1.runtime.IntakeHandler;
import org.firstinspires.ftc.teamcode.common.official.teleop.v1.runtime.MovementHandler;
import org.firstinspires.ftc.teamcode.common.official.teleop.v1.runtime.RoadrunnerHandler;
import org.firstinspires.ftc.teamcode.common.official.teleop.v1.runtime.SlideHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.revextensions2.ExpansionHubEx;
import org.sbs.bears.robotframework.controllers.DuckCarouselController;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeState;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "A - v1 teleop")
public class BlueTeleOp extends OpMode {


    /**
     * TeleOp States
     */
    public static volatile TeleOpRobotStates currentState = TeleOpRobotStates.STOPPED;
    private static Object stateMutex = new Object();

    /**
     * Controller Modes
     */
    public static Gamepad primaryGamepad;
    public static Gamepad secondaryGamepad;
    public static ControllerModes primaryControllerMode = ControllerModes.PRIMARY;
    public static double driveSpeedStrafe = 1;

    /**
     * Information Provisioning
     */
    public static boolean systemStopRequested = false;
    public static boolean objectInBucket = false;

    /**
     * Roadrunner Items
     */
    public static SampleMecanumDrive drive;
    public static FtcDashboard dashboard;

    /**
     * Stupid Michael MIT License: Open Source For Everyone
     */
    public static boolean isColorStripBlue = false;
    public static IntakeControllerRed redIntake;
    public static IntakeControllerBlue blueIntake;
    public static SlideController slideController;
    public static DuckCarouselController carouselController;

    /**
     * Run Time Applications
     */
    public static SlideHandler slideHandler = new SlideHandler();
    public static RoadrunnerHandler roadrunnerHandler = new RoadrunnerHandler();

    /**
     * Extra Components
     */
    public static RevColorSensorV3 bottomColorSensor;
    public static RevBlinkinLedDriver revBlinkinLedDriver;
    public static ExpansionHubEx expansionHubEx;
    public static MultipleTelemetry multipleTelemetry;

    /**
     * 线程池
     */
    private static HashMap<String, Thread> threadPool = new HashMap<>();

    /**
     * Interface Tag
     */
    public static String interfaceTag = "Tele Op";

    @Override
    public void init() {
        currentState = TeleOpRobotStates.INITIALIZING;
        multipleTelemetry = new MultipleTelemetry(telemetry);

        multipleTelemetry.clearAll();
        multipleTelemetry.addLine("Initializing...");
        multipleTelemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        primaryGamepad = gamepad1;
        secondaryGamepad = gamepad2;

        slideController = new SlideController(hardwareMap, telemetry);
        blueIntake = new IntakeControllerBlue(hardwareMap, slideController.blueDumperServo, telemetry);
        redIntake = new IntakeControllerRed(hardwareMap, slideController.redDumperServo, telemetry);
        carouselController = new DuckCarouselController(hardwareMap, telemetry);
        revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "rgb");
        bottomColorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        expansionHubEx = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 4");

        // LS: Light Sensor
        // PS: Proximity Sensor
        // MEAS: Measure Rate
        bottomColorSensor.write8(BroadcomColorSensor.Register.LS_MEAS_RATE, 0x01010000);
        bottomColorSensor.write8(BroadcomColorSensor.Register.PS_MEAS_RATE, 0x00000001);
        bottomColorSensor.write8(BroadcomColorSensor.Register.LS_GAIN, 0x00000000);

        redIntake.setState(IntakeState.DUMP);
        blueIntake.setState(IntakeState.DUMP);
        slideController.initTeleop();
        bottomColorSensor.setGain(Configuration.colorSensorGain);
        telemetry.update();

        blueIntake.dumperServo.setPosition(SlideController.dumperPosition_READY);
        drive.setPoseEstimate(new Pose2d(28.5, 65.5, 0));


        floodRuntimes();
        multipleTelemetry.clearAll();
        multipleTelemetry.addLine("Finished Init");
        multipleTelemetry.update();
    }

    private int initPass = 0;

    @Override
    public void init_loop() {
        if (initPass == 0) {
            revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            BlueTeleOp.isColorStripBlue = true;
            multipleTelemetry.clearAll();
            multipleTelemetry.addLine("Pending Start...");
            multipleTelemetry.update();
            initPass = 1;
        }

        for(Map.Entry<String, Thread> set : checkRuntimes().entrySet()) {
            if(set != null && !set.getValue().isAlive()) {
                startInterface(set.getKey());
            }
        }
    }

    @Override
    public void loop() {
        switch (currentState) {
            case STOPPED:
                telemetry.clearAll();
                telemetry.addLine("Robot Stopped");
                telemetry.update();
                break;

            case INITIALIZING:
                slideHandler.slideMovementEnabled = true;
                startThreadPool();
                synchronized (stateMutex) {
                    currentState = TeleOpRobotStates.RUNNING;
                }
                break;

                case RUNNING:
                    if(driveSpeedStrafe < 1) {
                        BlueTeleOp.revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                        isColorStripBlue = false;
                    } else if(!objectInBucket) {
                        if(!isColorStripBlue) {
                            BlueTeleOp.resetColor();
                            isColorStripBlue = true;
                        }
                    }

                    drive.update();
                    telemetry.addData("distance", redIntake.distanceSensor.getDistance(DistanceUnit.MM));
                    telemetry.update();
                    Log.d("DISTANCE", String.valueOf(redIntake.distanceSensor.getDistance(DistanceUnit.MM)));
                break;
        }
    }

    /**
     * Stop All Active Handler Threads
     */
    @Override
    public void stop() {
        synchronized (currentState) {
            currentState = TeleOpRobotStates.STOPPED;
        }
        roadrunnerHandler.sendKillSignal();
        killThreadPool();
    }

    /**
     * Flood Thread Pool
     */
    private static void floodRuntimes() {
        registerThread(ButtonHandler.primaryInterfaceTag, ButtonHandler.primaryRuntime);
        registerThread(ButtonHandler.secondaryInterfaceTag, ButtonHandler.secondaryRuntime);
        registerThread(IntakeHandler.interfaceTag, IntakeHandler.runtime);
        registerThread(MovementHandler.interfaceTag, MovementHandler.runtime);
    }

    /**
     * Check All Running Runtimes
     * @return a hashmap with the threads that are not running
     */
    private static HashMap<String, Thread> checkRuntimes() {
        HashMap<String, Thread> notRunning = new HashMap<>();
        for(Map.Entry<String, Thread> set : threadPool.entrySet()) {
            if(!set.getValue().isAlive()) {
                notRunning.put(set.getKey(), set.getValue());
            }
        }
        return notRunning;
    }

    /**
     * Iterate Over Thread Pool And Exit
     */
    private void killThreadPool() {
        Log.d(interfaceTag, "-------------------------------------------------------------------------------------------------");
        for (Map.Entry<String, Thread> set : threadPool.entrySet()) {
            set.getValue().interrupt();
            Log.i(interfaceTag, "Interrupted: " + set.getKey());
        }
        Log.d(interfaceTag, "-------------------------------------------------------------------------------------------------");
    }

    /**
     * Registers a thread to the thread pool
     *
     * @param interfaceTag the interface tag for the handler
     * @param thread       the runtime thread for the handler
     */
    public static void registerThread(String interfaceTag, Thread thread) {
        if (!threadPool.containsKey(interfaceTag)) {
            threadPool.put(interfaceTag, thread);
            Log.d(BlueTeleOp.interfaceTag, "Thread Registered: " + interfaceTag);
        } else {
            Log.d(BlueTeleOp.interfaceTag, "Thread Already Registered: " + interfaceTag);
        }
    }

    /**
     * Starts The Whole Thread Pool
     */
    public static void startThreadPool() {
        for (Map.Entry<String, Thread> set : threadPool.entrySet()) {
            if (!set.getValue().isAlive()) {
                set.getValue().start();
                Log.d(interfaceTag, "Started Process: " + set.getKey());
            }
        }
    }

    /**
     * Starts a specific interface handler
     * @param interfaceTag the interface handler tag
     */
    public static void startInterface(String interfaceTag) {
        Thread requestedThread = threadPool.get(interfaceTag);
        if (requestedThread != null && !requestedThread.isAlive()) {
            requestedThread.start();
            Log.d(interfaceTag, "Started Process: " + interfaceTag);
        }
    }

    /**
     * Stops a specific interface handler
     * @param interfaceTag the interface handler tag
     */
    public static void stopInterface(String interfaceTag) {
        Thread requestedThread = threadPool.get(interfaceTag);
        if (requestedThread != null) {
            requestedThread.interrupt();
        }
    }

    /**
     * Color Things
     */
    public static void resetColor() {
        revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

}
