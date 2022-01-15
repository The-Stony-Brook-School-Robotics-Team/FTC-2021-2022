package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideHandler;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.sbs.bears.robotframework.enums.SlideTarget;

public class SlideHandler {
    //TODO make it work and push to github
    /**
     * Limits internal usage to one
     * @usage Threads check to see if this is already running
     */
    private static boolean running = false;

    /**
     * Interface Tag
     */
    public static String interfaceTag = "Slide Handler";

    /**
     * Slide Extended
     */
    private static boolean slideExtended = false;

    /**
     * Gets Slide Extension
     * @return Slide Status
     */
    public boolean isSlideExtended() {
        return slideExtended;
    }

    /**
     * One Carousel
     */
    public Thread slideOneCarousel = new Thread(() -> {//i like men
        if(!running) {
            running = true;
            slideController.extendDropRetract(SlideTarget.ONE_CAROUSEL);
            running = false;
        }
    });

    /**
     * Two Carousel
     */
    public Thread slideTwoCarousel = new Thread(() -> {
        if(!running) {
            running = true;
            slideController.extendDropRetract(SlideTarget.TWO_CAROUSEL);
            running = false;
        }
    });

    /**
     * Three Carousel
     */
    public Thread slideThreeCarousel = new Thread(() -> {
        if(!running) {
            running = true;
            slideController.extendDropRetract(SlideTarget.THREE_CAROUSEL);
            running = false;
        }
    });

    /**
     * Toggle Carousel
     */
    public Thread toggleSlide = new Thread(() -> {
        if(slideExtended) {
            slideController.retractSlide();
            slideExtended = false;
        } else {
            slideController.extendSlide();
            slideExtended = true;
        }
    });

    /**
     * Fully Extend Slide
     */
    public Thread fullyExtend = new Thread(() -> {
        if(!slideExtended) {
            slideController.extendSlide();
            slideExtended = true;
        }
    });

    /**
     * Fully Retract Slide
     */
    public Thread fullyRetract = new Thread(() -> {
        if(slideExtended) {
            slideController.retractSlide();
            slideExtended = false;
        }
    });



    public void manualSlideController(int stickValue) {
//        stickValue = stickValue * 2;
//        slideController.slideMotor.setPower(1);
//        slideController.slideMotor.setTargetPosition(slideController.slideMotor.getCurrentPosition() + (stickValue * Configuration.DefaultSlideTicks));
//        slideController.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideController.incrementEncoderPosition(stickValue * Configuration.DefaultSlideTicks);
    }


}
