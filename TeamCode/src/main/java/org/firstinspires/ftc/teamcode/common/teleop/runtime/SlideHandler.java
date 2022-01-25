package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideController;
//import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideHandler;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.sbs.bears.robotframework.enums.SlideTarget;

public class SlideHandler {
    /**
     * Interface Tag
     */
    public static String interfaceTag = "Slide Handler";

    public boolean slideMovementEnabled = false;
    public boolean slideMoving = false;

    /**
     * Manual Slide Controller
     * @param stickValue increase multiplier for motor encoder position
     */
    public void manualSlideController(int stickValue) {
        if(slideMovementEnabled != true) { return; }
        if(!slideMoving) {
            slideMoving = true;
            stickValue = stickValue * -2;
            slideController.incrementEncoderPosition(stickValue * Configuration.DefaultSlideTicks);
            Log.d(interfaceTag, "Current Slide Motor Ticks: " + slideController.getSlideMotorPosition());
            slideMoving = false;
        }
    }

    public void DuckToTop() {
        if(slideMovementEnabled != true) { return; }
        if(!slideMoving) {
            slideMoving = true;
            slideController.extendDropRetract(SlideTarget.THREE_DEPOSIT);
            slideMoving = false;
        }
    }


}
