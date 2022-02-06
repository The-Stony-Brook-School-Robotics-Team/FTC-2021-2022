package org.firstinspires.ftc.teamcode.common.teleop;

import com.coyote.framework.core.geometry.Pose2d;

public class Configuration {

    /** Thresholding For Buttons */
    public static double rightStickXLimitTrigger = 0.02;

    /** Slide Thresholding */
    public static int DefaultSlideTicks = 25;
    public static double DefaultVerticalSlideIncrement = 0.01;

    /** Movement Adjustments */
    public static double SlowMovementMultiplier = 0.2;

    /** Roadrunner Movement Config */
    public static int inchesLeft = 12;
    public static int inchesRight = 12;
    public static int inchesForward = 12;
    public static int inchesBack = 12;

    /** Thread Management */
    public static int maxInternalThreads = 5;
    public static int maxRuntimeThreads = 10;
    public static boolean poolingEnabled = true;

    /** Color Sensor Settings */
    public static int colorSensorGain = 10;
    public static double colorSensorWhiteAlpha = 0.8;


}
