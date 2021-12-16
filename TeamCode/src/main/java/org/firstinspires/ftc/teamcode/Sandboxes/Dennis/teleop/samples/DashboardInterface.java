package org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.samples;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.misc.Beta;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.jetbrains.annotations.NotNull;

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

    private final int maxLines = 20;
    private volatile String[] dashboardTelemetryLines = new String[maxLines];
    private boolean handlerCraftingPacket = false;


    /**
     * Add a line into the dashboard telemetry
     */
    @Beta
    public void writeLine(@NotNull String message) {
        /**
         * Find any duplicate lines and set the flag off if there are any
         */
        boolean foundDupeFlag = false;
        for(int i = 0; i < dashboardTelemetryLines.length; i++) {
            if(message.equals(dashboardTelemetryLines[i])) {
                if(foundDupeFlag == false) {
                    foundDupeFlag = true;
                }
                // Pass through
                continue;
            }
        }
        /**
         * If the flag isn't thrown, find the next null object and replace it with the string
         */
        if(foundDupeFlag == false) {
            boolean foundNullFlag = false; // null flag
            int nullParamIndex = -1; // the index of that null object in the array
            for(int i = 0; i < dashboardTelemetryLines.length; i++) {
                if(dashboardTelemetryLines[i] == null) {
                    foundNullFlag = true; // set off the flag
                    nullParamIndex = i; // set the index
                    break; // we break here because we only need to find the first occurrence of this object
                } else {
                    continue; // continue until there is a flag thrown
                }
            }
            /**
             * If the flag is thrown, and we don't have the default index, we set the message into
             * the index of the first null object occurrence
             */
            if(nullParamIndex != -1 && foundNullFlag == true) {
                dashboardTelemetryLines[nullParamIndex] = message; // nullParamIndex is the index of the first null occurrence
            }
        }
    }

    /**
     * Delete Line in the dashboard telemetry
     * @param line
     */
    @Beta
    public void deleteLine(@NotNull String line) {
        /**
         * Loop through the array until the line shows up, and then set it null
         */
        for(int i = 0; i < dashboardTelemetryLines.length; i++  ) {
            if(dashboardTelemetryLines[i] == line) {
                dashboardTelemetryLines[i] = null;
            } else {
                break;
            }
        }
    }

//    /**
//     * The actual handler thats writing the data into the dashboard
//     */
//    @Beta
//    public Thread dashboardHandler = new Thread(() -> {
//        /**
//         * Handle the telemetry interface
//         */
//        while(currentState.equals(TeleOpRobotStates.RUNNING) || currentState.equals(TeleOpRobotStates.AUTONOMOUS)) {
//            /**
//             * Generate the robots relative position on the map
//             */
//            TelemetryPacket telemetryPacket = new TelemetryPacket();
//            Canvas ftcField = telemetryPacket.fieldOverlay();
//            DashboardUtil.drawRobot(ftcField, drive.getPoseEstimate());
//            /**
//             * Go through every telemetry line and add it to the packet
//             */
//            handlerCraftingPacket = true; // this is just to prevent over-writing values as we pass through the array
//            for(int i = 0; i < dashboardTelemetryLines.length; i++) {
//                /**
//                 * Because the array may or may not contain null objects, we want to check for them
//                 */
//                if(dashboardTelemetryLines[i] != null) {
//                    telemetryPacket.addLine(dashboardTelemetryLines[i]);
//                }
//            }
//            /**
//             * Turn the flag off to allow for writing of the packet
//             */
//            handlerCraftingPacket = false;
//            /**
//             * Finally send the packet to the dashboard
//             */
//            dashboard.sendTelemetryPacket(telemetryPacket);
//        }
//    });



}
