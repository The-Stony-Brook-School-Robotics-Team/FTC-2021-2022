package org.firstinspires.ftc.teamcode.common.teleop;

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

@TeleOp(name = "A - Red TeleOp")
public class RedTeleOp extends OpMode {


    /**
     * Teleop States
     */
    public static volatile TeleOpRobotStates currentState = TeleOpRobotStates.STOPPED;
    private static Object stateMutex = new Object();

    /**
     * Controller Modes
     */
    public static Gamepad primaryGamepad;
    public static Gamepad secondaryGamepad;
    public static ControllerModes primaryControllerMode = ControllerModes.PRIMARY;
    public static ControllerModes secondaryControllerMode = ControllerModes.PRIMARY;
    public static double driveSpeedStrafe = 1;

    /**
     * Information Provisioning
     */
    public static double systemRuntime = 0;
    public static boolean systemStopRequested = false;

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
    public static MovementHandler movementHandler = new MovementHandler();
    public static ButtonHandler buttonHandler = new ButtonHandler();
    public static SlideHandler slideHandler = new SlideHandler();
    public static IntakeHandler intakeHandler = new IntakeHandler();
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

        /**
         * Roadrunner initialization
         * */
        drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        this.primaryGamepad = gamepad1;
        this.secondaryGamepad = gamepad2;


        // TODO: IF ANYTHING IS NOT WORKING CHECK THIS
        ButtonHandler.invertedDucKSpinner = true;



        /**
         * Initialization
         */
        slideController = new SlideController(hardwareMap, telemetry);
        blueIntake = new IntakeControllerBlue(hardwareMap, slideController.blueDumperServo, telemetry);
        redIntake = new IntakeControllerRed(hardwareMap, slideController.redDumperServo, telemetry);
        carouselController = new DuckCarouselController(hardwareMap, telemetry);
        revBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "rgb");
        bottomColorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
        expansionHubEx = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 4");

        /**
         * Increase Read Speeds
         */
        // LS: Light Sensor
        // PS: Proximity Sensor
        // MEAS: Measure Rate
        bottomColorSensor.write8(BroadcomColorSensor.Register.LS_MEAS_RATE, 0x01010000);
        bottomColorSensor.write8(BroadcomColorSensor.Register.PS_MEAS_RATE, 0x00000001);
        bottomColorSensor.write8(BroadcomColorSensor.Register.LS_GAIN, 0x00000000);

        /**
         * Configuration
         */

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

        /**
         * Pass through the runtimes and make sure that they are running
         */
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
                movementHandler.movementEnabled = true;
                slideHandler.slideMovementEnabled = true;
                startThreadPool();
                synchronized (stateMutex) {
                    currentState = TeleOpRobotStates.RUNNING;
                }
                break;

            case RUNNING:

                if(!threadPool.get(MovementHandler.interfaceTag).isAlive()) {
                    threadPool.get(MovementHandler.interfaceTag).start();
                }



                int numThreads = threadPool.size();
                multipleTelemetry.addLine("---------------------------------------------------------");
                multipleTelemetry.addData("Current # Threads Running: ", numThreads);
                multipleTelemetry.addLine("---------------------------------------------------------");
                multipleTelemetry.addData("Drive Handler Thread Status: ", threadPool.get(MovementHandler.interfaceTag).isAlive());
                multipleTelemetry.addData("Intake Handler Thread Status: ", threadPool.get(IntakeHandler.interfaceTag).isAlive());
                multipleTelemetry.addData("(Primary) Button Handler Thread Status: ", threadPool.get(ButtonHandler.primaryInterfaceTag).isAlive());
                multipleTelemetry.addData("(Secondary) Button Handler Thread Status: ", threadPool.get(ButtonHandler.secondaryInterfaceTag).isAlive());
                multipleTelemetry.addLine("---------------------------------------------------------");
                multipleTelemetry.addData("Drive Handler Thread State: ", threadPool.get(MovementHandler.interfaceTag).getState());
                multipleTelemetry.addData("Intake Handler Thread State: ", threadPool.get(IntakeHandler.interfaceTag).getState());
                multipleTelemetry.addData("(Primary) Button Handler Thread State: ", threadPool.get(ButtonHandler.primaryInterfaceTag).getState());
                multipleTelemetry.addData("(Secondary) Button Handler Thread State: ", threadPool.get(ButtonHandler.secondaryInterfaceTag).getState());
                multipleTelemetry.addLine("---------------------------------------------------------");
                multipleTelemetry.update();
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
     * Lists every thread in the thread pool
     */
    public static void listThreadPool() {
        Log.d(interfaceTag, "-------------------------------------------------------------------------------------------------");
        for (Map.Entry<String, Thread> set : threadPool.entrySet()) {
            Log.d(interfaceTag, "Thread Name: " + set.getKey());
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
            if (set.getValue().isAlive() == false) {
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
