package org.firstinspires.ftc.teamcode.common.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.teleop.enums.ControllerModes;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.common.teleop.misc.Beta;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;


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

        /**
        if(!dashboardHandler.isAlive()) {
            dashboardHandler.start();
        }
        */

        switch(currentState) {
            case STOPPED:
                telemetry.clearAll();
                telemetry.addLine("robot stopped");
                telemetry.update();
                break;

            case RUNNING:
                telemetry.addLine("robot running, runtime: " + this.getRuntime());
                telemetry.update();
                break;

        }

    }



    public Thread buttonHandlerRuntime = new Thread(() -> {
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
     * Working Roadrunner Handler Runtime
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

//            Pose2d poseEstimate = drive.getPoseEstimate();
//            TelemetryPacket telemetryPacket = new TelemetryPacket();
//            Canvas ftcField = telemetryPacket.fieldOverlay();
//            DashboardUtil.drawRobot(ftcField, poseEstimate);
//            // Telemetry Packet Update
//            telemetryPacket.put("Estimated Pose X", poseEstimate.getX());
//            telemetryPacket.put("Estimated Pose Y", poseEstimate.getY());
//            telemetryPacket.put("Estimated Pose Heading", poseEstimate.getHeading());
//            dashboard.sendTelemetryPacket(telemetryPacket);

            drive.update();
        }
    });

    /**
     * Old TeleOp Roadrunner Handler
     */
    @Deprecated
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

        }
    });


}
