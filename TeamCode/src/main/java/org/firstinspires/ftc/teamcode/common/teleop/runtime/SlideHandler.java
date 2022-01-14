package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideHandler;

import android.util.Log;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.sbs.bears.robotframework.enums.SlideTarget;

public class SlideHandler {

    /**
     * Limits internal usage to one
     * @usage Threads check to see if this is already running
     */
    private static boolean running = false;

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
    public Thread slideOneCarousel = new Thread(() -> {
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
        slideController.incrementEncoderPosition((stickValue + 1) * Configuration.DefaultSlideTicks);
        Log.d("SLIDE CONTROLLER", String.valueOf((stickValue + 1) * Configuration.DefaultSlideTicks));

    }


}
