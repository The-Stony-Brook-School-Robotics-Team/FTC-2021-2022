package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;

public class IntakeHandler {

    /**
     * Interface Tag
     */
    public static String interfaceTag = "Intake Handler";

    public static boolean objectInBucket = false;

    public Thread runtime = new Thread(() -> {
        while(!BlueTeleOp.systemStopRequested && BlueTeleOp.currentState == TeleOpRobotStates.RUNNING || BlueTeleOp.currentState == TeleOpRobotStates.INITIALIZING) {
            /**
             * Check Buckets / Intakes
             */
            BlueTeleOp.redIntake.checkIntake();
            BlueTeleOp.blueIntake.checkIntake();

            /**
             * Update Color
             */
            if(BlueTeleOp.redIntake.isObjectInPayload() == true || BlueTeleOp.blueIntake.isObjectInPayload() == true) {
                objectInBucket = true;
                BlueTeleOp.revBlinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                BlueTeleOp.isColorStripBlue = false;
            } else {
                objectInBucket = false;
            }
        }
    });

}
