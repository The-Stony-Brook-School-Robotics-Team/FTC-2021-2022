package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;

public class IntakeHandler {

    /**
     * Interface Tag
     */
    public static String interfaceTag = "Intake Handler";

    public Thread runtime = new Thread(() -> {
        while(!OfficialTeleop.systemStopRequested && OfficialTeleop.currentState == TeleOpRobotStates.RUNNING || OfficialTeleop.currentState == TeleOpRobotStates.INITIALIZING) {
            /**
             * Check Buckets / Intakes
             */
            OfficialTeleop.redIntake.checkIntake();
            OfficialTeleop.blueIntake.checkIntake();
            OfficialTeleop.slideController.checkForBucketObject();

            /**
             * Update Color
             */
            if(OfficialTeleop.redIntake.isObjectInPayload() == true || OfficialTeleop.blueIntake.isObjectInPayload() == true) {
                OfficialTeleop.revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            }

            Log.d(interfaceTag, "Interface Still Running");
        }
    });

}
