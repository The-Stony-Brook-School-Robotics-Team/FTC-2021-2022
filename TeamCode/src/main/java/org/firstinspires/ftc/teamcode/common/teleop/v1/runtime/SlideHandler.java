package org.firstinspires.ftc.teamcode.common.teleop.v1.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.v1.BlueTeleOp.slideController;
//import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideHandler;

import android.util.Log;

import org.firstinspires.ftc.teamcode.common.teleop.v1.Configuration;

public class SlideHandler {
    /**
     * Interface Tag
     */
    public static String interfaceTag = "Slide Handler";

    /**
     * Safety Checks
     */
    public boolean slideMovementEnabled = false;
    public boolean slideMoving = false;

    /**
     * Manual Slide Controller
     * @param stickValue increase multiplier for motor encoder position
     */
    public void manualSlideController(int stickValue) {
        /**
         * Check if slide movement is enabled
         */
        if (!slideMovementEnabled) {
            return;
        }

        if(slideMoving) {
            return;
        }

        /**
         * Reset The Capstone Logic
         */
        if (ButtonHandler.currentSegmentPosition != ButtonHandler.SegmentPositions.EXTEND) {
            ButtonHandler.currentSegmentPosition = ButtonHandler.SegmentPositions.EXTEND;
        }

        slideMoving = true;
        stickValue = stickValue * -2;
        slideController.incrementEncoderPosition(stickValue * Configuration.DefaultSlideTicks, true);
        Log.d(interfaceTag, "Current Slide Motor Ticks: " + slideController.getSlideMotorPosition());
        slideMoving = false;
    }

    public void resetSlideEncoder()
    {
            slideController.resetEncoder();
            Log.d("SlideController","Reset Encoder to 0");
    }
}
