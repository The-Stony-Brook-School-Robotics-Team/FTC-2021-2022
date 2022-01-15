package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.currentState;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.drive;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.gamepad;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slowModeToggled;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.common.teleop.misc.Beta;

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
    public Thread runtime = new Thread(() -> {
        while(currentState.equals(TeleOpRobotStates.RUNNING)) {
            switch (currentRunType) {
                case SLOW:
                    if(!slowDriving.isAlive()) {
                        if(defaultDriving.isAlive()) {
                            defaultDriving.interrupt();
                        }
                        if(sprintDriving.isAlive()) {
                            sprintDriving.interrupt();
                        }
                        slowDriving.start();
                    }
                    break;
                case DEFAULT:
                    if(!defaultDriving.isAlive()) {
                        if(slowDriving.isAlive()) {
                            slowDriving.interrupt();
                        }
                        if(sprintDriving.isAlive()) {
                            sprintDriving.interrupt();
                        }
                        defaultDriving.start();
                    }
                    break;
                case SPRINT:
                    if(!sprintDriving.isAlive()) {
                        if(slowDriving.isAlive()) {
                            slowDriving.interrupt();
                        }
                        if(defaultDriving.isAlive()) {
                            defaultDriving.interrupt();
                        }
                        sprintDriving.start();
                    }
                    break;
                default:
                    Log.e("Movement Handler", ": Internal Logic Error @runtime -> Drive Handler Checks");
                    Log.e(interfaceTag, "Log Info: " + currentRunType);
            }
        }
    });


    private static Thread sprintDriving = new Thread(() -> {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.getLeftY(),
                        -gamepad.getLeftX(),
                        -gamepad.getRightX()
                )
        );
        drive.update();
    });

    private static Thread defaultDriving = new Thread(() -> {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.getLeftY(),
                        -gamepad.getLeftX(),
                        -gamepad.getRightX()
                )
        );
        drive.update();
    });

    private static Thread slowDriving = new Thread(() -> {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.getLeftY(),
                        -gamepad.getLeftX(),
                        -gamepad.getRightX()
                )
        );
        drive.update();
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
}
