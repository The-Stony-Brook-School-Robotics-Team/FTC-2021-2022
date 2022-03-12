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
        if(slideMovementEnabled != true) { return; }
        /**
         * Reset The Capstone Logic
         */
        if(buttonHandler.currentSegmentPosition != ButtonHandler.SegmentPositions.EXTEND) {
            buttonHandler.currentSegmentPosition = ButtonHandler.SegmentPositions.EXTEND;
        }
        /**
         * Check if the slide is already moving
         */
        if(!slideMoving) {
            slideMoving = true;
            stickValue = stickValue * -2;
            slideController.incrementEncoderPosition(stickValue * Configuration.DefaultSlideTicks, true);
            Log.d(interfaceTag, "Current Slide Motor Ticks: " + slideController.getSlideMotorPosition());
            slideMoving = false;
        }
    }

    public AtomicReference<Boolean> hasFinishedReset = new AtomicReference<>();
    public boolean hasStartedMagswitchRoutine = false;


    // MARC & MICHAEL ADDED @ COMP
    public void resetSlideEncoder()
    {

            slideController.resetEncoder();
            Log.d("SlideController","Reset Encoder to 0");
    }

    /**
     * Deposit (B)
     */
    public void DuckToTop() {
        if(slideMovementEnabled != true) { return; }
        if(!slideMoving) {
            slideMoving = true;
            slideController.extendDropRetract(SlideTarget.TOP_DEPOSIT);
            slideMoving = false;
        }
    }


}
