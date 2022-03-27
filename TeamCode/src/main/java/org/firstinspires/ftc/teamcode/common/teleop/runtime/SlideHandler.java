package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.buttonHandler;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.slideController;
//import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideHandler;

import android.util.Log;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.enums.SlideTarget;

import java.util.concurrent.atomic.AtomicReference;

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
