package org.firstinspires.ftc.teamcode.common.teleop;

public class Configuration {

    /** Thresholding For Buttons */
    public static double rightStickXLimitTrigger = 0.02; // This stops random triggering
    public static double rightStickXLimitThreshold = 0.05; // This makes it so that the joystick has to pass a threshold to move the slide
    public static double staticSlideAdjustmentMultiplier = 1.1; // This multiplies slide adjustment speed

    /** Slide Thresholding */
    public static int MaxSlideTicks = 1000;
    public static int DefaultSlideTicks = 20;
    public static double DefaultVerticalSlideIncrement = 0.05;

}
