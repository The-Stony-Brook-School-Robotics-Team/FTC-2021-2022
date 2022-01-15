package org.firstinspires.ftc.teamcode.common.teleop;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name = "AX - Qualifier One TeleOp", group = "default")
public class OfficialTeleop extends OpMode {


    /**
     * Teleop States
     */
    public static volatile TeleOpRobotStates currentState = TeleOpRobotStates.STOPPED;
    private static Object stateMutex = new Object();

    /** Controller Modes */
    public static GamepadEx gamepad;
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
        gamepad = new GamepadEx(gamepad1);

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
        Log.d(interfaceTag, ": Tele Op Ready");

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

                redIntake.checkIntake();
                blueIntake.checkIntake();

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
            threadPool.put(movementHandler.interfaceTag, movementHandler.runtime);
            Log.d(interfaceTag, "Thread Registered: " + movementHandler.interfaceTag);
        }
        if(!buttonHandler.runtime.isAlive()) {
            buttonHandler.runtime.start();
            threadPool.put(buttonHandler.interfaceTag, buttonHandler.runtime);
            Log.d(interfaceTag, "Thread Registered: " + buttonHandler.interfaceTag);
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
            Log.d(interfaceTag, "Interrupted: " + set.getKey());
        }
    }
}
