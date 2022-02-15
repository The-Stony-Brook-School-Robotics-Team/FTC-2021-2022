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

    public static boolean objectInBucket = false;

    public Thread runtime = new Thread(() -> {
        while(!OfficialTeleop.systemStopRequested && OfficialTeleop.currentState == TeleOpRobotStates.RUNNING || OfficialTeleop.currentState == TeleOpRobotStates.INITIALIZING) {
            /**
             * Check Buckets / Intakes
             */
            OfficialTeleop.redIntake.checkIntake();
            OfficialTeleop.blueIntake.checkIntake();

            /**
             * Update Color
             */
            if(OfficialTeleop.redIntake.isObjectInPayload() == true || OfficialTeleop.blueIntake.isObjectInPayload() == true) {
                objectInBucket = true;
                OfficialTeleop.revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                OfficialTeleop.isColorStripBlue = false;
            } else {
                objectInBucket = false;
            }
        }
    });

}
