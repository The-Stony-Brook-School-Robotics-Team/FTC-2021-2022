package org.firstinspires.ftc.teamcode.common.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe.SlideController;
import org.firstinspires.ftc.teamcode.common.teleop.enums.ControllerModes;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.common.teleop.misc.Beta;
import org.firstinspires.ftc.teamcode.common.teleop.samples.DashboardInterface;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.sbs.bears.robotframework.controllers.IntakeController;
import org.sbs.bears.robotframework.enums.IntakeSide;
import org.sbs.bears.robotframework.enums.IntakeState;


@TeleOp(name="A - TeleOp Qualifier One", group="default")
public class TeleOpRoutine extends OpMode {

    /**
     * Teleop States
     * */
    public static volatile TeleOpRobotStates currentState = TeleOpRobotStates.STOPPED;
    private static Object stateMutex = new Object();

    /** Controller Modes */
    private static GamepadEx gamepad;
    ControllerModes controllerMode = ControllerModes.PRIMARY;

    /** Toggle Indicators */
    private static boolean slowmodeToggled = false; // TODO: Add slowmode code reader in the teleop
    private static boolean linearslideToggled = false; // TODO: Add a handler to handle this function if toggled
    private static boolean autonomousToggled = false; // TODO: Add handler to determine if this is enabled

    /** Roadrunner Items */
    public static SampleMecanumDrive drive;
    public static FtcDashboard dashboard;

    /** Stupid Michael MIT License: Open Source For Everyone */
    private static IntakeController redIntake;
    private static IntakeController blueIntake;
    private static SlideController slideController;

    /** Internal Usage */
    private static Telemetry internalTelemetry;
    private static DcMotor spoolMotor = null;

    @Override
    public void init() {
        currentState = TeleOpRobotStates.INITIALIZING;
        /**
         * Roadrunner initialization
         * */
        drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        internalTelemetry = telemetry;
        spoolMotor  = hardwareMap.get(DcMotor.class, "spool");
        gamepad = new GamepadEx(gamepad1);

        redIntake = new IntakeController(hardwareMap, telemetry, IntakeSide.RED);
        blueIntake = new IntakeController(hardwareMap, telemetry, IntakeSide.BLUE);
        slideController = new SlideController(hardwareMap, telemetry);

        /**
         * Update current state to continue
         */
        currentState = TeleOpRobotStates.RUNNING;
    }

    @Override
    public void loop() {
        /**
         * Runtime Initialization
         */
        if(!buttonHandlerRuntime.isAlive()) {
            buttonHandlerRuntime.start();
        }
        if(!roadrunnerHandlerRuntime.isAlive()) {
            roadrunnerHandlerRuntime.start();
        }
        // if(!DashboardInterface.dashboardInterfaceUpdater.isAlive()) {
        //     DashboardInterface.dashboardInterfaceUpdater.start();
        // }

        switch(currentState) {
            case STOPPED:
                telemetry.clearAll();
                telemetry.addLine("robot stopped");
                telemetry.update();
                break;

            case RUNNING:
                // Check Intakes
                redIntake.checkIntake();
                blueIntake.checkIntake();
                // End Intakes



                telemetry.addLine("robot running, runtime: " + this.getRuntime());
                telemetry.update();
                break;

        }

    }



    public Thread buttonHandlerRuntime = new Thread(() -> {
        while(currentState == TeleOpRobotStates.RUNNING) {
            if(gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                controllerMode = ControllerModes.SECONDARY;
            } else {
                controllerMode = ControllerModes.PRIMARY;
            }

            switch(controllerMode) {
                case PRIMARY:
                    // A
                    if(gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                        // TODO: Add open claw function
                    } else {
                        // TODO: Add ensure claw is closed function
                    }
                    // X
                    if(gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                        slowmodeToggled = !slowmodeToggled;
                    }
                    // Y
                    if(gamepad.isDown(GamepadKeys.Button.Y)) {
                        // TODO: Add duck spinner function
                        telemetry.addData("button down!", 1);
                        telemetry.update();
                    }
                    // RB
                    if(gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        // TODO: Add toggle linear slide modes {IN, OUT}
                    }
                    // Left dpad
                    if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                        if(redIntake.getState() == IntakeState.PARK) {
                            redIntake.setState(IntakeState.BASE);
                        } else {
                            redIntake.setState(IntakeState.PARK);
                        }
                    }
                    // Right dpad
                    if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                        if(blueIntake.getState() == IntakeState.PARK) {
                            blueIntake.setState(IntakeState.BASE);
                        } else {
                            blueIntake.setState(IntakeState.PARK);
                        }
                    }


                    break;
                case SECONDARY:
                    slideMotionControl();
                    break;
            }
        }

    });


    /**
     * Working Roadrunner Handler Runtime
     */
    @Beta
    private static Thread roadrunnerHandlerRuntime = new Thread(() -> {
        while(currentState.equals(TeleOpRobotStates.RUNNING)) {
            // Set Weighted Power
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad.getLeftY(),
                            -gamepad.getLeftX(),
                            -gamepad.getRightX()
                    )
            );
            drive.update();
        }
    });

    private static void slideMotionControl() {
        double joystickPosition = gamepad.getRightY();
        internalTelemetry.addData("Joystick Position", joystickPosition);
        internalTelemetry.addData("encoder pos", spoolMotor.getCurrentPosition());
        internalTelemetry.update();

        int cPos  = spoolMotor.getCurrentPosition();
        cPos = cPos + (int)joystickPosition;
        spoolMotor.setTargetPosition(cPos);
    }




}
