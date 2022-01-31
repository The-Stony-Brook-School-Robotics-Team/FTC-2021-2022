package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.currentState;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.drive;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.gamepad;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.movementHandler;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;

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
    public static volatile boolean movementEnabled = true;

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
     * Autonomous Flags
     */
    public static boolean autonomousRunning = false;

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
        if(!movementEnabled) {
            movementEnabled = true;
        }
    }

    /**
     * Disable Drivign
     */
    public static void disableDriving() {
        if(movementEnabled) {
            movementEnabled = false;
        }
    }

}

/**
 * Driver Movement
 */
class MovementHandlers {
    public static String interfaceTag = "Movement Handlers";

    public static Thread sprintDriving = new Thread(MovementHandlers::sprintRunner);
    public static Thread defaultDriving = new Thread(MovementHandlers::defaultRunner);
    public static Thread slowDriving = new Thread(MovementHandlers::slowRunner);

    private static void sprintRunner() {
        if (!MovementHandler.movementEnabled) {
            drive.setWeightedDrivePower(new Pose2d());
            return;
        }
        if (MovementHandler.autonomousRunning) {
            drive.setWeightedDrivePower(new Pose2d());
            return;
        }
        if (MovementHandler.currentDriverMode != MovementHandler.DriverMode.DRIVER) {
            drive.setWeightedDrivePower(new Pose2d());
            return;
        }
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_x,
                        gamepad.left_stick_y,
                        -gamepad.right_stick_x
                )
        );
        drive.update();
    }

    private static void defaultRunner() {
        if (!MovementHandler.movementEnabled) {
            return;
        }
        if (MovementHandler.autonomousRunning) {
            return;
        }
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_x,
                        gamepad.left_stick_y,
                        -gamepad.right_stick_x
                )
        );
        drive.update();
    }

    private static void slowRunner() {
        if (!MovementHandler.movementEnabled) {
            drive.setWeightedDrivePower(new Pose2d());
            return;
        }
        if (MovementHandler.autonomousRunning) {
            drive.setWeightedDrivePower(new Pose2d());
            return;
        }
        if (MovementHandler.currentDriverMode != MovementHandler.DriverMode.DRIVER) {
            drive.setWeightedDrivePower(new Pose2d());
            return;
        }
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y * Configuration.SlowMovementMultiplier,
                        gamepad.left_stick_x * Configuration.SlowMovementMultiplier,
                        -gamepad.right_stick_x * Configuration.SlowMovementMultiplier
                )
        );
        drive.update();
    }
}