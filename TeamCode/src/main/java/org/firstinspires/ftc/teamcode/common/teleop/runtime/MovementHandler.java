package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.currentState;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.drive;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.gamepad;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.common.teleop.misc.Beta;

import java.util.HashMap;

public class MovementHandler {

    // TODO: Add functionality
    public enum RunType {
        SLOW,
        DEFAULT,
        SPRINT
    }

    /**
     * Interface Tag
     */
    public static String interfaceTag = "Movement Handler";

    /**
     * Current Driving Run Type
     */
    private static RunType currentRunType = RunType.DEFAULT;

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

}

class MovementHandlers {
    public static Thread sprintDriving = new Thread(() -> {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x,
                        -gamepad.right_stick_x
                )
        );
        drive.update();
    });

    public static Thread defaultDriving = new Thread(() -> {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x,
                        -gamepad.right_stick_x
                )
        );
        drive.update();
    });

    public static Thread slowDriving = new Thread(() -> {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x,
                        -gamepad.right_stick_x
                )
        );
        drive.update();
    });
}