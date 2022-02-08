package org.firstinspires.ftc.teamcode.common.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.teleop.enums.ControllerModes;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.common.teleop.runtime.ButtonHandler;
import org.firstinspires.ftc.teamcode.common.teleop.runtime.IntakeHandler;
import org.firstinspires.ftc.teamcode.common.teleop.runtime.MovementHandler;
import org.firstinspires.ftc.teamcode.common.teleop.runtime.RoadrunnerHandler;
import org.firstinspires.ftc.teamcode.common.teleop.runtime.SlideHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
    public static NormalizedColorSensor normalizedColorSensor;
    public static RevBlinkinLedDriver revBlinkinLedDriver;

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
        blueIntake = new IntakeControllerBlue(hardwareMap, telemetry);
        slideController = new SlideController(hardwareMap, telemetry);
        carouselController = new DuckCarouselController(hardwareMap, telemetry);
        normalizedColorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "rgb");

        /**
         * Configuration
         */
        slideController.initTeleop();
        redIntake.setState(IntakeState.PARK);
        blueIntake.setState(IntakeState.PARK);
        normalizedColorSensor.setGain(Configuration.colorSensorGain);
        telemetry.update();

        floodRuntimes();
    }

    @Override
    public void init_loop() {
        revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }

    @Override
    public void loop() {
        switch(currentState) {
            case STOPPED:
                telemetry.clearAll();
                telemetry.addLine("robot stopped");
                telemetry.update();
                break;
            case INITIALIZING:
                movementHandler.movementEnabled = true;
                slideHandler.slideMovementEnabled = true;
                startThreadPool();

                synchronized (stateMutex) { currentState = TeleOpRobotStates.RUNNING; }
                break;

            case RUNNING:
                /**
                 * Bucket Logic
                 */
                if(slideController.teleopIsObjectInBucket()) {
                    Log.d(interfaceTag, "Set servo to closed");
                    slideController.dumperServo.setPosition(slideController.dumperPosition_CLOSED);
                }
               telemetry.addData("blue Distance Sensor", blueIntake.distanceSensor.getDistance(DistanceUnit.MM));
                telemetry.addData("red Distance Sensor", blueIntake.distanceSensor.getDistance(DistanceUnit.MM));
                telemetry.addData("Red Servo Position", redIntake.getServoPos());
                telemetry.addData("Blue Servo Position", blueIntake.getServoPos());
                telemetry.update();

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
