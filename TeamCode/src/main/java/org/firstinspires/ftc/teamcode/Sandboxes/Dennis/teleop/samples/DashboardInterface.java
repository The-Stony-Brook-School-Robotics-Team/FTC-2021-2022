package org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.samples;

import org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.misc.Beta;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Turns out this code worked, but the dashboard alreadt has handling for multiple packets
 */
@Beta
@Deprecated
public class DashboardInterface {

    /**
     * FTC Dashboard Interface For Elements In TeleOp
     * @param message
     */

    public static final int maxLines = 20;
    public static volatile HashMap<Integer, String> dashboardTelemetry = new HashMap<>();
    public static boolean handlerCraftingPacket = false;


    /**
     * Add a line into the dashboard telemetry
     */
    @Beta
    @Deprecated
    public static void writeLine(@NotNull Integer index, @NotNull String message) {
        if(dashboardTelemetry.containsValue(message)) {
            dashboardTelemetry.forEach((identifier, string) -> {
                if(string == message) {
                    dashboardTelemetry.put(identifier, message);
                }
            });
        } else {
            dashboardTelemetry.put(index, message);
        }
    }

    @Beta
    @Deprecated
    public static void deleteLine(@NotNull Integer index, @NotNull String line) {

    }

    public static void deleteLine(@NotNull Integer index) {

    }


}
