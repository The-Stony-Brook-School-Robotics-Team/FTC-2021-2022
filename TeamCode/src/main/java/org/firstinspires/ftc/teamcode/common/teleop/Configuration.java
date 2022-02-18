package org.firstinspires.ftc.teamcode.common.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class Configuration {

    /** Thresholding For Buttons */
    public static double rightStickXLimitTrigger = 0.02;

    public static double leftTriggerTreshold = 0.02;
    public static double rightTriggerTreshold = 0.02;

    /** Slide Thresholding */
    public static int DefaultSlideTicks = 25;
    public static double DefaultVerticalSlideIncrement = .003;//0.01;

    
    /** Movement Adjustments */
    public static double SlowMovementStrafeMultiplier = 0.3;

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
    public static double colorSensorWhiteAlpha = 0.001; // was 0.8

    /**
     * Movement Control
     */


}
