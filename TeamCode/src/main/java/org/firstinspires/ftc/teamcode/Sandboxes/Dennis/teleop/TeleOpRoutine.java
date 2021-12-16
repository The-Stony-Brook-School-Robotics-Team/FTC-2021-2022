package org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop;

import static java.lang.String.valueOf;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.enums.ControllerModes;
import org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.misc.Beta;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;

@TeleOp(name="A - TeleOp", group="default")
public class TeleOpRoutine extends OpMode {

    /**
     * Teleop States
     * */
    public static volatile TeleOpRobotStates currentState = TeleOpRobotStates.STOPPED;
    public static Object stateMutex = new Object();

    /** Controller Modes */
    public static GamepadEx gamepad;
    ControllerModes controllerMode = ControllerModes.PRIMARY;

    /** Toggle Indicators */
    private boolean slowmodeToggled = false; // TODO: Add slowmode code reader in the teleop
    private boolean linearslideToggled = false; // TODO: Add a handler to handle this function if toggled
    private boolean autonomousToggled = false; // TODO: Add handler to determine if this is enabled

    /** Roadrunner Items */
    public static SampleMecanumDrive drive;
    public static FtcDashboard dashboard;

    @Override
    public void init() {
        currentState = TeleOpRobotStates.INITIALIZING;
        /**
         * Roadrunner initialization
         * */
        drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        gamepad = new GamepadEx(gamepad1);

        /**
         * Runtime Initialization
         */
        buttonHandlerRuntime.start();
        roadrunnerHandlerRuntime.start();
        dashboardHandler.start();
        /**
         * Update current state to continue
         */
        currentState = TeleOpRobotStates.RUNNING;
    }

    @Override
    public void loop() {
        switch(currentState) {
            case STOPPED:
                telemetry.clearAll();
                telemetry.addLine("robot stopped");
                telemetry.update();
                break;

            case RUNNING:
                synchronized (stateMutex) { currentState = TeleOpRobotStates.RUNNING; }
                handleRoadrunner();
                telemetry.addLine("robot running, runtime: " + this.getRuntime());
                telemetry.update();
                break;

        }

    }



    public Thread buttonHandlerRuntime = new Thread(() -> {
        // button states


        // checks if robot is running or not
        while(currentState == TeleOpRobotStates.RUNNING) {
            // check left bumper state +
            if(gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                controllerMode = ControllerModes.SECONDARY;
            } else if(gamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
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
                    // B
                    // TODO: TBD
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
                    // RT
                    // TODO: TALK WITH TIGER ABT THIS ONE
                    // RB
                    if(gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        // TODO: Add toggle linear slide modes {IN, OUT}
                    }
                    break;
                case SECONDARY:
                    if(gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                        // start teleop auton
                    }

                    if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                        // strafe left
                    }

                    if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                        // strafe right
                    }

                    if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                        // forward
                    }

                    if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                        // back
                    }


                    break;
            }
        }

    });

    public Thread lightHandlerRuntime = new Thread(() -> {

    });



    /**
     * Roadrunner Handler
     */
    @Beta
    public Thread roadrunnerHandlerRuntime = new Thread(() -> {
        while(currentState.equals(TeleOpRobotStates.RUNNING)) {
            // Set Weighted Power
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad.getLeftY(),
                            -gamepad.getLeftX(),
                            -gamepad.getRightX()
                    )
            );

            Pose2d poseEstimate = drive.getPoseEstimate();
            TelemetryPacket telemetryPacket = new TelemetryPacket();
            Canvas ftcField = telemetryPacket.fieldOverlay();
            DashboardUtil.drawRobot(ftcField, poseEstimate);
            // Telemetry Packet Update
            telemetryPacket.put("Estimated Pose X", poseEstimate.getX());
            telemetryPacket.put("Estimated Pose Y", poseEstimate.getY());
            telemetryPacket.put("Estimated Pose Heading", poseEstimate.getHeading());
            dashboard.sendTelemetryPacket(telemetryPacket);
            drive.update();
        }
    });

    /**
     * Current Working Roadrunner Runtime
     */
    public void handleRoadrunner() {
        // Set Weighted Power
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.getLeftY(),
                        -gamepad.getLeftX(),
                        -gamepad.getRightX()
                )
        );

        Pose2d poseEstimate = drive.getPoseEstimate();
//        TelemetryPacket telemetryPacket = new TelemetryPacket();
//        Canvas ftcField = telemetryPacket.fieldOverlay();
//        DashboardUtil.drawRobot(ftcField, poseEstimate);
//        // Telemetry Packet Update
//        telemetryPacket.put("Estimated Pose X", poseEstimate.getX());
//        telemetryPacket.put("Estimated Pose Y", poseEstimate.getY());
//        telemetryPacket.put("Estimated Pose Heading", poseEstimate.getHeading());
//        dashboard.sendTelemetryPacket(telemetryPacket);

        writeLine("X: " + poseEstimate.getX());
        writeLine("Y: " + poseEstimate.getY());
        writeLine("R: " + poseEstimate.getHeading());
        drive.update();
    }

    /**
     * FTC Dashboard Interface For Elements In TeleOp
     * @param message
     */

    private final int maxLines = 20;
    private volatile String[] dashboardTelemetryLines = new String[maxLines];
    private boolean handlerCraftingPacket = false;


    /**
     * Add a line into the dashboard telemetry
     */
    @Beta
    public void writeLine(@NotNull String message) {
        /**
         * Find any duplicate lines and set the flag off if there are any
         */
        boolean foundDupeFlag = false;
        for(int i = 0; i < dashboardTelemetryLines.length; i++) {
            if(message.equals(dashboardTelemetryLines[i])) {
                if(foundDupeFlag == false) {
                    foundDupeFlag = true;
                }
                // Pass through
                continue;
            }
        }
        /**
         * If the flag isn't thrown, find the next null object and replace it with the string
         */
        if(foundDupeFlag == false) {
            boolean foundNullFlag = false; // null flag
            int nullParamIndex = -1; // the index of that null object in the array
            for(int i = 0; i < dashboardTelemetryLines.length; i++) {
                if(dashboardTelemetryLines[i] == null) {
                    foundNullFlag = true; // set off the flag
                    nullParamIndex = i; // set the index
                    break; // we break here because we only need to find the first occurrence of this object
                } else {
                    continue; // continue until there is a flag thrown
                }
            }
            /**
             * If the flag is thrown, and we don't have the default index, we set the message into
             * the index of the first null object occurrence
             */
            if(nullParamIndex != -1 && foundNullFlag == true) {
                dashboardTelemetryLines[nullParamIndex] = message; // nullParamIndex is the index of the first null occurrence
            }
        }
    }

    /**
     * Delete Line in the dashboard telemetry
     * @param line
     */
    @Beta
    public void deleteLine(@NotNull String line) {
        /**
         * Loop through the array until the line shows up, and then set it null
         */
        for(int i = 0; i < dashboardTelemetryLines.length; i++  ) {
            if(dashboardTelemetryLines[i] == line) {
                dashboardTelemetryLines[i] = null;
            } else {
                break;
            }
        }
    }

    /**
     * The actual handler thats writing the data into the dashboard
     */
    @Beta
    public Thread dashboardHandler = new Thread(() -> {
        /**
         * Handle the telemetry interface
         */
        while(currentState.equals(TeleOpRobotStates.RUNNING) || currentState.equals(TeleOpRobotStates.AUTONOMOUS)) {
            /**
             * Generate the robots relative position on the map
             */
            TelemetryPacket telemetryPacket = new TelemetryPacket();
            Canvas ftcField = telemetryPacket.fieldOverlay();
            DashboardUtil.drawRobot(ftcField, drive.getPoseEstimate());
            /**
             * Go through every telemetry line and add it to the packet
             */
            handlerCraftingPacket = true; // this is just to prevent over-writing values as we pass through the array
            for(int i = 0; i < dashboardTelemetryLines.length; i++) {
                /**
                 * Because the array may or may not contain null objects, we want to check for them
                 */
                if(dashboardTelemetryLines[i] != null) {
                    telemetryPacket.addLine(dashboardTelemetryLines[i]);
                }
            }
            /**
             * Turn the flag off to allow for writing of the packet
             */
            handlerCraftingPacket = false;
            /**
             * Finally send the packet to the dashboard
             */
            dashboard.sendTelemetryPacket(telemetryPacket);
        }
    });

}
