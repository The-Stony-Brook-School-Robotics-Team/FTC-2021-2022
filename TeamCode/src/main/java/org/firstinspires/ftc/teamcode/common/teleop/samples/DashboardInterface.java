package org.firstinspires.ftc.teamcode.common.teleop.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.common.teleop.misc.Beta;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.jetbrains.annotations.NotNull;

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
    /**
     * Finds the next null in an string linesay
     * @param lines The string linesay to iterate
     * @return Index of the null object
     */
    public static int findNull(String[] lines) {
        boolean flag = false;
        int index = -1;
        int i = 0;
        while (!flag) {
            if(i > lines.length - 1) {
                flag = !flag;
            } else {
                if(lines[i] == null) {
                    System.out.println(String.format("flag at index %s, object: %s", i, lines[i]));
                    index = i;
                    flag = !flag;
                }
            }
            i++;
        }
        return index;
    }

    /**
     * Finds the value in an linesay
     * @param value The string value to find
     * @param lines The string linesay to iterate
     * @return Index of the string object
     */
    public static int findValue(String value, String[] lines) {
        boolean flag = false;
        int index = -1;
        int i = 0;
        while(!flag) {
            if(i > lines.length - 1) {
                flag = !flag;
            } else {
                if(lines[i] == value) {
                    System.out.println(String.format("flag at index %s, object: %s", i, lines[i]));
                    index = i;
                    flag = !flag;
                }
            }
            i++;
        }
        return index;
    }

    /**
     * Writes a line to the string linesay
     * @param line The string value to write
     * @return Final string linesay
     */
    public static String[] writeLine(String line) {
        // Check for short string
        if(line == "") {
            return lines;
        }
        // Find null index
        int index = findNull(lines);
        if(index != -1) {
            lines[index] = line;
            return lines;
        } else {
            return lines;
        }
    }

    /**
     * Writes a line to the string linesay
     * @param index The index of the string (starts at 1)
     * @param line The string value to write
     * @return Final string linesay
     */
    public static String[] writeLine(Integer index, String line) {
        // Check for short string or invalid index
        if(line == "" || index > lines.length) {
            return lines;
        }
        lines[index - 1] = line;
        return lines;
    }


    /**
     * Deletes a line from the string linesay
     * @param line The string value to delete
     * @return Final string linesay
     */
    public static String[] deleteLine(String line) {
        // Check for short string
        if(line == "") {
            return lines;
        }
        // Find null index
        int index = findValue(line, lines);
        lines[index] = null;
        return lines;
    }

    /**
     * Deletes a line from the string linesay
     * @param index The index of the string
     * @return Final string linesay
     */
    public static String[] deleteLine(Integer index) {
        // Check for short string or invalid index
        if(index > lines.length) {
            return lines;
        }
        lines[index] = null;
        return lines;
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
