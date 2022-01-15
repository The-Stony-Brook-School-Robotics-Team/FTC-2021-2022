package org.firstinspires.ftc.teamcode.common.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.teleop.enums.ControllerModes;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.common.teleop.runtime.ButtonHandler;
import org.firstinspires.ftc.teamcode.common.teleop.runtime.MovementHandler;
import org.firstinspires.ftc.teamcode.common.teleop.runtime.SlideHandler;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.controllers.DuckCarouselController;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.SlideTarget;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "A - Qualifier One TeleOp", group = "default")
public class OfficialTeleop extends OpMode {


    /**
     * Teleop States
     */
    public static volatile TeleOpRobotStates currentState = TeleOpRobotStates.STOPPED;
    private static Object stateMutex = new Object();

    /** Controller Modes */
    public static Gamepad gamepad;
    public static ControllerModes controllerMode = ControllerModes.PRIMARY;

    /** Toggle Indicators */
    public static boolean slowModeToggled = false; // TODO: Add slowmode code reader in the teleop

    /**
     * Information Provisioning
     */
    public static double systemRuntime = 0;

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

    /**
     * Thread Pool
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
        this.gamepad = gamepad1;

        // redIntake = new IntakeControllerRed(hardwareMap, telemetry);
        // blueIntake = new IntakeControllerBlue(hardwareMap, telemetry);
        slideController = new SlideController(hardwareMap, telemetry);
        carouselController = new DuckCarouselController(hardwareMap, telemetry);

        floodRuntimes();
        /**
         * Update current state to continue
         */
        currentState = TeleOpRobotStates.RUNNING;
    }

    @Override
    public void init_loop() {
        floodRuntimes();
    }

    @Override
    public void loop() {
        floodRuntimes();


        switch(currentState) {
            case STOPPED:
                telemetry.clearAll();
                telemetry.addLine("robot stopped");
                telemetry.update();
                break;

            case RUNNING:

                // redIntake.checkIntake();
                // blueIntake.checkIntake();

                systemRuntime = getRuntime();
                telemetry.update();
                break;

        }

    }

    /**
     * Stop All Active Handler Threads
     */
    @Override
    public void stop() {
        exitThreads();
    }

    /**
     * Flood Thread Pool
     */
    private static void floodRuntimes() {
        if(!movementHandler.runtime.isAlive()) {
            movementHandler.runtime.start();
            if(!threadPool.containsKey(movementHandler)) {
                threadPool.put(movementHandler.interfaceTag, movementHandler.runtime);
                Log.d(interfaceTag, "Thread Registered: " + movementHandler.interfaceTag);
            } else {
                Log.d(interfaceTag, "Thread: " + movementHandler.interfaceTag + " exists");
            }

        }
        if(!buttonHandler.runtime.isAlive()) {
            buttonHandler.runtime.start();
            if(threadPool.containsKey(buttonHandler.interfaceTag)) {
                threadPool.put(buttonHandler.interfaceTag, buttonHandler.runtime);
                Log.d(interfaceTag, "Thread Registered: " + buttonHandler.interfaceTag);
            } else {
                Log.d(interfaceTag, "Thread: " + buttonHandler.interfaceTag + " exists");
            }

        }
    }

    /**
     * Iterate Over Thread Pool And Exit
     */
    private void exitThreads() {
        for(Map.Entry<String, Thread> set : threadPool.entrySet()) {
            if(set.getValue().isAlive()) {
                set.getValue().interrupt();
            }
            Log.i(interfaceTag, "Interrupted: " + set.getKey());
        }
    }
}
