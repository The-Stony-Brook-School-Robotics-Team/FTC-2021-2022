package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.currentState;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.drive;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.driveSpeedStrafe;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.isColorStripBlue;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.primaryGamepad;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.common.teleop.misc.Beta;

public class MovementHandler {

    /**
     * Run Type for Movement
     */
    public enum RunType {
        DEFAULT
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
                case DEFAULT:
                    if(!MovementHandlers.defaultDriving.isAlive()) {
                        MovementHandlers.defaultDriving.start();
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
        MovementHandlers.defaultDriving.interrupt();
    }

}

/**
 * Driver Movement
 */
class MovementHandlers {


    public static String interfaceTag = "Movement Handlers";
    public static Thread defaultDriving = new Thread(MovementHandlers::defaultRunner);

    private static void defaultRunner() {
        if (!MovementHandler.movementEnabled) {
            return;
        }
        if (MovementHandler.autonomousRunning) {
            return;
        }
        if(driveSpeedStrafe < 1) {
            BlueTeleOp.revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            isColorStripBlue = false;
        } else if(BlueTeleOp.objectInBucket) {
            if(!isColorStripBlue) {
            BlueTeleOp.resetColor();
                isColorStripBlue = true;
            }
        }

        drive.setWeightedDrivePower(
                new Pose2d(
                        -primaryGamepad.left_stick_x * driveSpeedStrafe,
                        primaryGamepad.left_stick_y * driveSpeedStrafe,
                        -primaryGamepad.right_stick_x * driveSpeedStrafe
                )
        );
        drive.update();
    }
}