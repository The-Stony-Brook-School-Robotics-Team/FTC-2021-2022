package org.firstinspires.ftc.teamcode.common.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.teleop.enums.ControllerModes;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.common.teleop.runtime.ButtonHandler;
import org.firstinspires.ftc.teamcode.common.teleop.runtime.IntakeHandler;
import org.firstinspires.ftc.teamcode.common.teleop.runtime.MovementHandler;
import org.firstinspires.ftc.teamcode.common.teleop.runtime.RoadrunnerHandler;
import org.firstinspires.ftc.teamcode.common.teleop.runtime.SlideHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.revextensions2.ExpansionHubEx;
import org.sbs.bears.robotframework.controllers.DuckCarouselController;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeState;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "A - Qualifier One TeleOp")
public class OfficialTeleop extends OpMode {


    /**
     * Teleop States
     */
    public static volatile TeleOpRobotStates currentState = TeleOpRobotStates.STOPPED;
    private static Object stateMutex = new Object();

    /** Controller Modes */
    public static Gamepad primaryGamepad;
    public static Gamepad secondaryGamepad;
    public static ControllerModes primaryControllerMode = ControllerModes.PRIMARY;
    public static ControllerModes secondaryControllerMode = ControllerModes.PRIMARY;
    public static double driveSpeed = 1;

    /**
     * Information Provisioning
     */
    public static double systemRuntime = 0;
    public static boolean systemStopRequested = false;

    /** Roadrunner Items */
    public static SampleMecanumDrive drive;
    public static FtcDashboard dashboard;

    /** Stupid Michael MIT License: Open Source For Everyone */
    public static IntakeControllerRed redIntake;
    public static IntakeControllerBlue blueIntake;
    public static SlideController slideController;
    public static DuckCarouselController carouselController;

    /** Run Time Applications */
    public static MovementHandler movementHandler = new MovementHandler();
    public static ButtonHandler buttonHandler = new ButtonHandler();
    public static SlideHandler slideHandler = new SlideHandler();
    public static IntakeHandler intakeHandler = new IntakeHandler();
    public static RoadrunnerHandler roadrunnerHandler = new RoadrunnerHandler();

    /** Extra Components */
    public static RevColorSensorV3 bottomColorSensor;
    public static RevBlinkinLedDriver revBlinkinLedDriver;
    public static ExpansionHubEx expansionHubEx;

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
        /**
         * Roadrunner initialization
         * */
        drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        this.primaryGamepad = gamepad1;
        this.secondaryGamepad = gamepad2;

        /**
         * Initialization
         */
        redIntake = new IntakeControllerRed(hardwareMap, telemetry);

        slideController = new SlideController(hardwareMap, telemetry);
        blueIntake = new IntakeControllerBlue(hardwareMap, slideController.dumperServo, telemetry);
        carouselController = new DuckCarouselController(hardwareMap, telemetry);
        revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "rgb");
        bottomColorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        expansionHubEx = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 4");

        /**
         * Increase Read Speeds
         */
        bottomColorSensor.write8(BroadcomColorSensor.Register.LS_MEAS_RATE,0x01010000);
        bottomColorSensor.write8(BroadcomColorSensor.Register.PS_MEAS_RATE,0x00000001);


        /**
         * Configuration
         */
        slideController.initTeleop();
        redIntake.setState(IntakeState.DUMP);
        blueIntake.setState(IntakeState.DUMP);
        bottomColorSensor.setGain(Configuration.colorSensorGain);
        telemetry.update();

        blueIntake.dumperServo.setPosition(SlideController.dumperPosition_READY);
        drive.setPoseEstimate(new Pose2d(28.5, 65.5, 0));
        floodRuntimes();
    }

    private int initPass = 0;
    @Override
    public void init_loop() {
        if(initPass == 0) {
            revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            initPass = 1;
        }
    }

    @Override
    public void loop() {
        switch(currentState) {
            case STOPPED:
                telemetry.clearAll();
                telemetry.addLine("Robot Stopped");
                telemetry.update();
                break;

            case INITIALIZING:
                movementHandler.movementEnabled = true;
                slideHandler.slideMovementEnabled = true;
                startThreadPool();
                synchronized (stateMutex) { currentState = TeleOpRobotStates.RUNNING; }
                break;

            case RUNNING:
                telemetry.addData("Bottom Color Sensor Alpha: ", bottomColorSensor.alpha());
                telemetry.addData("Bottom Color Sensor Light Detected: ", bottomColorSensor.getLightDetected());
                telemetry.addData("I2C Draw (A): ", expansionHubEx.getI2cBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
                telemetry.addData("I2C Draw (mA): ", expansionHubEx.getI2cBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS));
                telemetry.update();
                break;

            case DEBUG:
                break;
        }
    }

    /**
     * Stop All Active Handler Threads
     */
    @Override
    public void stop() {
        synchronized (currentState) { currentState = TeleOpRobotStates.STOPPED; }
        movementHandler.sendKillSignal();
        roadrunnerHandler.sendKillSignal();
        killThreadPool();
    }

    /**
     * Flood Thread Pool
     */
    private static void floodRuntimes() {
        registerThread(buttonHandler.primaryInterfaceTag, buttonHandler.primaryRuntime);
        registerThread(buttonHandler.secondaryInterfaceTag, buttonHandler.secondaryRuntime);
        registerThread(movementHandler.interfaceTag, movementHandler.runtime);
        registerThread(intakeHandler.interfaceTag, intakeHandler.runtime);
    }

    /**
     * Iterate Over Thread Pool And Exit
     */
    private void killThreadPool() {
        Log.d(interfaceTag, "-------------------------------------------------------------------------------------------------");
        for(Map.Entry<String, Thread> set : threadPool.entrySet()) {
            set.getValue().interrupt();
            Log.i(interfaceTag, "Interrupted: " + set.getKey());
        }
        Log.d(interfaceTag, "-------------------------------------------------------------------------------------------------");
    }

    /**
     * Lists every thread in the thread pool
     */
    public static void listThreadPool() {
        Log.d(interfaceTag, "-------------------------------------------------------------------------------------------------");
        for(Map.Entry<String, Thread> set : threadPool.entrySet()) {
            Log.d(interfaceTag, "Thread Name: " + set.getKey());
        }
        Log.d(interfaceTag, "-------------------------------------------------------------------------------------------------");
    }

    /**
     * Registers a thread to the thread pool
     * @param interfaceTag the interface tag for the handler
     * @param thread the runtime thread for the handler
     */
    public static void registerThread(String interfaceTag, Thread thread) {
        if(!threadPool.containsKey(interfaceTag)) {
            threadPool.put(interfaceTag, thread);
            Log.d(OfficialTeleop.interfaceTag, "Thread Registered: " + interfaceTag);
        } else {
            Log.d(OfficialTeleop.interfaceTag, "Thread Already Registered: " + interfaceTag);
        }
    }

    /**
     * Starts The Whole Thread Pool
     */
    public static void startThreadPool() {
        for(Map.Entry<String, Thread> set : threadPool.entrySet()) {
            if(set.getValue().isAlive() == false) {
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
        if(requestedThread != null && !requestedThread.isAlive()) {
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
        if(requestedThread != null) {
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
