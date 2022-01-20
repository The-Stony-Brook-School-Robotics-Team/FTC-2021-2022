package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.DrivingEnabled;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.currentState;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.drive;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.gamepad;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.common.teleop.misc.Beta;

public class MovementHandler {

    /**
     * Run Type for Movement
     */
    public enum RunType {
        SLOW,
        DEFAULT,
        SPRINT
    }

    public enum DriverMode {
        AUTOMATIC,
        DRIVER
    }

    /**
     * Movement Disabled
     */
    public static boolean MovementEnabled = true;

    /**
     * Interface Tag
     */
    public static String interfaceTag = "Movement Handler";

    /**
     * Current Driving Run Type
     */
    private static RunType currentRunType = RunType.DEFAULT;
    public static DriverMode currentDriverMode = DriverMode.DRIVER;

    /**
     * Working Movement Handler Runtime
     */
    @Beta
    public static Thread runtime = new Thread(() -> {
        while(currentState.equals(TeleOpRobotStates.RUNNING) || currentState.equals(TeleOpRobotStates.INITIALIZING)) {
            switch (currentRunType) {
                case SLOW:
                    if(!MovementHandlers.slowDriving.isAlive()) {
                        if(MovementHandlers.defaultDriving.isAlive()) {
                            MovementHandlers.defaultDriving.interrupt();
                        }
                        if(MovementHandlers.sprintDriving.isAlive()) {
                            MovementHandlers.sprintDriving.interrupt();
                        }
                        MovementHandlers.slowDriving.start();
                    }
                    break;
                case DEFAULT:
                    if(!MovementHandlers.defaultDriving.isAlive()) {
                        if(MovementHandlers.slowDriving.isAlive()) {
                            MovementHandlers.slowDriving.interrupt();
                        }
                        if(MovementHandlers.sprintDriving.isAlive()) {
                            MovementHandlers.sprintDriving.interrupt();
                        }
                        MovementHandlers.defaultDriving.start();
                    }
                    break;
                case SPRINT:
                    if(!MovementHandlers.sprintDriving.isAlive()) {
                        if(MovementHandlers.slowDriving.isAlive()) {
                            MovementHandlers.slowDriving.interrupt();
                        }
                        if(MovementHandlers.defaultDriving.isAlive()) {
                            MovementHandlers.defaultDriving.interrupt();
                        }
                        MovementHandlers.sprintDriving.start();
                    }
                    break;
                default:
                    Log.e(interfaceTag, ": Internal Logic Error @runtime -> Drive Handler Checks");
                    Log.e(interfaceTag, "Log Info: " + currentRunType);
            }
        }
    });




    /**
     * Lets you set the current run type
     * @param runType the robot drive type
     */
    public static void setRunType(RunType runType) {
        currentRunType = runType;
    }

    /**
     * Gets the current movement run type
     * @return current robot run type
     */
    public static RunType getRunType() {
        return currentRunType;
    }

    /**
     * Kill Threads
     */
    public static void sendKillSignal() {
        MovementHandler.runtime.interrupt();
        MovementHandlers.slowDriving.interrupt();
        MovementHandlers.sprintDriving.interrupt();
        MovementHandlers.defaultDriving.interrupt();
    }

    /**
     * Enable Driving
     */
    public static void enableDriving() {
        if(DrivingEnabled != true) {
            DrivingEnabled = true;
        }
    }

    /**
     * Disable Drivign
     */
    public static void disableDriving() {
        if(DrivingEnabled != false) {
            DrivingEnabled = false;
        }
    }

}

/**
 * Brings Automatic Movement To TeleOp
 */
class RoadrunnerHandlers {

    /**
     * Movement Types to Schedule
     */
    enum MovementTypes {
        EMPTY(0),
        LEFT(Configuration.inchesLeft),
        RIGHT(Configuration.inchesRight),
        FORWARD(Configuration.inchesForward),
        BACK(Configuration.inchesBack);

        private int inches;

        private int getInches() {
            return this.inches;
        }

        private MovementTypes(int inches) {
            this.inches = inches;
        }

    }

    private static MovementTypes scheduledMovement = null;
    private static String interfaceTag = "RoadRunner Handler";

    /**
     * Kill all management threads
     */
    public static void sendKillSignal() {
        movementExecutor.interrupt();
    }

    /**
     * Internal executor
     */
    private static Thread movementExecutor = new Thread(() -> {
        switch(scheduledMovement) {
            case LEFT:
                scheduledMovement = MovementTypes.EMPTY;
                Trajectory left = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeLeft(scheduledMovement.getInches())
                        .build();
                drive.followTrajectory(left);
                Log.d(interfaceTag, "Moving " + String.valueOf(scheduledMovement.getInches()) + " inches left");
                break;
            case RIGHT:
                scheduledMovement = MovementTypes.EMPTY;
                Trajectory right = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeRight(scheduledMovement.getInches())
                        .build();
                drive.followTrajectory(right);
                Log.d(interfaceTag, "Moving " + String.valueOf(scheduledMovement.getInches()) + " right left");
                break;

            case FORWARD:
                scheduledMovement = MovementTypes.EMPTY;
                Trajectory forward = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(scheduledMovement.getInches())
                        .build();
                drive.followTrajectory(forward);
                Log.d(interfaceTag, "Moving " + String.valueOf(scheduledMovement.getInches()) + " inches forward");
                break;
            case BACK:
                scheduledMovement = MovementTypes.EMPTY;
                Trajectory back = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(scheduledMovement.getInches())
                        .build();
                drive.followTrajectory(back);
                Log.d(interfaceTag, "Moving " + String.valueOf(scheduledMovement.getInches()) + " inches backwards");
                break;

            default:
                requestKill();
                break;
        }
    });

    /** Schedule A Movement */
    // TODO: Add an indicator showing if the robot took the movement
    public static void scheduleMovement(MovementTypes movementType) {
        // Check if the drive is busy
        if(drive.isBusy()) { return; }
        // Check if the executor is alive, if not start it
        if(!movementExecutor.isAlive()) {
            movementExecutor.start();
        }
        // Schedule the requested movement
        scheduledMovement = movementType;
    }

    /** Kills Movement Executor Thread */
    private static void requestKill() {
        movementExecutor.interrupt();
    }

}

/**
 * Driver Movement
 */
class MovementHandlers {
    public static String interfaceTag = "Movement Handlers";

    public static Thread sprintDriving = new Thread(() -> {
        if(MovementHandler.MovementEnabled) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad.left_stick_y,
                            -gamepad.left_stick_x,
                            -gamepad.right_stick_x
                    )
            );
            drive.update();
            Log.d(interfaceTag, "Sprint Driving Inactive");
        } else {
            drive.setWeightedDrivePower(new Pose2d());
        }
    });

    public static Thread defaultDriving = new Thread(() -> {
        if(MovementHandler.MovementEnabled) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad.left_stick_y,
                            -gamepad.left_stick_x,
                            -gamepad.right_stick_x
                    )
            );
            drive.update();
            Log.d(interfaceTag, "Default Driving Inactive");
        } else {
            drive.setWeightedDrivePower(new Pose2d());
        }
    });

    public static Thread slowDriving = new Thread(() -> {
        if(MovementHandler.MovementEnabled) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad.left_stick_y * Configuration.SlowMovementMultiplier,
                            -gamepad.left_stick_x * Configuration.SlowMovementMultiplier,
                            -gamepad.right_stick_x * Configuration.SlowMovementMultiplier
                    )
            );
            drive.update();
            Log.d(interfaceTag, "Slow Driving Active");
        } else {
            drive.setWeightedDrivePower(new Pose2d());
            Log.d(interfaceTag, "Slow Driving Inactive");
        }
    });
}