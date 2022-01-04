package org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.misc.Beta;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Turns out this code worked, but the dashboard alreadt has handling for multiple packets
 */
@Beta
@Deprecated
public class DashboardInterface {

    private static boolean initialized = false;
    private static boolean displayRobotPosition = false;

    /**
     * FTC Dashboard Interface For Elements In TeleOp
     * @param message
     */

    public static final int maxLines = 20;
    public static volatile String[] lines = new String[maxLines];
    public static boolean handlerCraftingPacket = false;

    /**
     * Items for drawing the robot
     */
    private static SampleMecanumDrive internalDrive = null;
    private static FtcDashboard internalDashboard = FtcDashboard.getInstance();


    public DashboardInterface(@NotNull SampleMecanumDrive drive) {
        internalDrive = drive;
        initialized = true;
    }


    /**
     * Add a line into the dashboard telemetry
     */
    @Beta
    public static void writeLine(@NotNull Integer index, @NotNull String message) {
    }

    @Beta
    public static void deleteLine(@NotNull Integer index, @NotNull String line) {

    }


    public static void deleteLine(@NotNull Integer index) {

    }


    /**
     * Starts the updater thread
     * @return DASHBOARD_INTERFACE_UPDATER_FLAGS FLAG
     */
    public static DASHBOARD_INTERFACE_UPDATER_FLAGS start() {
        if (initialized && !dashboardInterfaceUpdater.isAlive()) {
            try {
                dashboardInterfaceUpdater.checkAccess();
            } catch (Exception ex) {
                return DASHBOARD_INTERFACE_UPDATER_FLAGS.NO_ACCESS;
            }
            dashboardInterfaceUpdater.start();
            return DASHBOARD_INTERFACE_UPDATER_FLAGS.RUNNING;
        } else if(initialized && dashboardInterfaceUpdater.isAlive()) {
            return DASHBOARD_INTERFACE_UPDATER_FLAGS.RUNNING;
        }
        return DASHBOARD_INTERFACE_UPDATER_FLAGS.COULD_NOT_START;
    }

    /**
     * Stops the updater thread
     * @return DASHBOARD_INTERFACE_UPDATER_FLAGS FLAG
     */
    public static DASHBOARD_INTERFACE_UPDATER_FLAGS stop() {
        if (initialized && dashboardInterfaceUpdater.isAlive()) {
            try {
                dashboardInterfaceUpdater.checkAccess();
            } catch (Exception ex) {
                return DASHBOARD_INTERFACE_UPDATER_FLAGS.NO_ACCESS;
            }
            dashboardInterfaceUpdater.stop();
            return DASHBOARD_INTERFACE_UPDATER_FLAGS.STOPPED;
        } else if(initialized && !dashboardInterfaceUpdater.isAlive()) {
            return DASHBOARD_INTERFACE_UPDATER_FLAGS.STOPPED;
        }
        return DASHBOARD_INTERFACE_UPDATER_FLAGS.COULD_NOT_STOP;
    }

    /**
     * Toggle the dashboard telemetry for position
     * @return
     */
    public static boolean togglePositionTelemety() {
        displayRobotPosition = !displayRobotPosition;
        return displayRobotPosition;
    }


    public enum DASHBOARD_INTERFACE_UPDATER_FLAGS {
        STOPPED,
        RUNNING,
        ERROR,
        COULD_NOT_START,
        COULD_NOT_STOP,
        NO_ACCESS
    }

    private static Thread dashboardInterfaceUpdater = new Thread(() -> {
            // Gets last pose
            Pose2d poseEstimate = internalDrive.getPoseEstimate();

            // Crafts a new packet
            TelemetryPacket telemetryPacket = new TelemetryPacket();

            // Draws the robot field and cavas
            Canvas ftcField = telemetryPacket.fieldOverlay();
            DashboardUtil.drawRobot(ftcField, poseEstimate);

            // Display robot position in dashboard telemetry
            if(displayRobotPosition) {
                telemetryPacket.put("Estimated Pose X", poseEstimate.getX());
                telemetryPacket.put("Estimated Pose Y", poseEstimate.getY());
                telemetryPacket.put("Estimated Pose Heading", poseEstimate.getHeading());
            }

            for(int i = 0; i < maxLines; i++) {
                if(lines[i] != null) {
                    telemetryPacket.put(String.format("%i", i), lines[i]);
                }
            }

            // Send the packet
            internalDashboard.sendTelemetryPacket(telemetryPacket);
    });

}
