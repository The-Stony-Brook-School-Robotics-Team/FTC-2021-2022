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


    public static void start() {

    }

    public static void stop() {

    }

    public static boolean togglePositionTelemety() {

    }

    private static Thread dashboardInterfaceUpdater = new Thread(() -> {
            Pose2d poseEstimate = internalDrive.getPoseEstimate();
            TelemetryPacket telemetryPacket = new TelemetryPacket();
            Canvas ftcField = telemetryPacket.fieldOverlay();
            DashboardUtil.drawRobot(ftcField, poseEstimate);


            telemetryPacket.put("Estimated Pose X", poseEstimate.getX());
            telemetryPacket.put("Estimated Pose Y", poseEstimate.getY());
            telemetryPacket.put("Estimated Pose Heading", poseEstimate.getHeading());
            internalDashboard.sendTelemetryPacket(telemetryPacket);


    });

}
