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

    public static boolean slideMoving = false;

    /**
     * Manual Slide Controller
     * @param stickValue increase multiplier for motor encoder position
     */
    public void manualSlideController(int stickValue) {
        if(!slideMoving) {
            MovementHandler.disableDriving();
            slideMoving = true;
            stickValue = stickValue * -2;
            slideController.incrementEncoderPosition(stickValue * Configuration.DefaultSlideTicks);
            Log.d(interfaceTag, "Current Slide Motor Ticks: " + slideController.getSlideMotorPosition());
            MovementHandler.enableDriving();
            slideMoving = false;
        }
    }

    public static void DuckToMiddle() {
        if(!slideMoving) {
            slideMoving = true;
            //slideController.extendDropRetract(SlideTarget.TWO_CAROUSEL);
            slideController.extendSlide();
            slideController.dropCube();
            slideController.retractSlide();

            slideMoving = false;
        }
    }


}
