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
    public static boolean slowmodeToggled = false; // TODO: Add slowmode code reader in the teleop
    public boolean linearslideToggled = false; // TODO: Add a handler to handle this function if toggled
    public boolean autonomousToggled = false; // TODO: Add handler to determine if this is enabled

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



    @Override
    public void init() {
        currentState = TeleOpRobotStates.INITIALIZING;
        /**
         * Roadrunner initialization
         * */
        drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        gamepad = new GamepadEx(gamepad1);

        redIntake = new IntakeControllerRed(hardwareMap, telemetry);
        blueIntake = new IntakeControllerBlue(hardwareMap, telemetry);
        slideController = new SlideController(hardwareMap, telemetry);
        carouselController = new DuckCarouselController(hardwareMap, telemetry);

        /**
         * Update current state to continue
         */
        currentState = TeleOpRobotStates.RUNNING;
    }



    @Override
    public void loop() {
        if(!movementHandler.runtime.isAlive()) {
            movementHandler.runtime.start();
        }
        if(!buttonHandler.runtime.isAlive()) {
            buttonHandler.runtime.start();
        }

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


}
