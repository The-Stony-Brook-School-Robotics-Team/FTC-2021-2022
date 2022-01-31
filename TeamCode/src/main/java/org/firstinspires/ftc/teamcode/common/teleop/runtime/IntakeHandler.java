package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;

public class IntakeHandler {

    /**
     * Interface Tag
     */
    public static String interfaceTag = "Intake Handler";

    public Thread runtime = new Thread(() -> {
        while(!OfficialTeleop.systemStopRequested && OfficialTeleop.currentState == TeleOpRobotStates.RUNNING || OfficialTeleop.currentState == TeleOpRobotStates.INITIALIZING) {
            OfficialTeleop.redIntake.checkIntake();
            OfficialTeleop.blueIntake.checkIntake();

            OfficialTeleop.slideController.checkForBucketObject();


        }
    });


}
