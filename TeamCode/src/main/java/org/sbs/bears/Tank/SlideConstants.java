package org.sbs.bears.Tank;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SlideConstants {

    public static double claw_OPEN = 0.29; //claw was flipped around and re-mounted; new values entirely -cs
    public static double claw_IDLE = 0.36;
    public static double claw_CLOSED = 0.0;


    public static int slideMotorPosition_PARKED = 5;
    public static int slideMotorPosition_THREE_PRELOAD = 696;
    public static int slideMotorPosition_TWO_PRELOAD = 508;
    public static int slideMotorPosition_ONE_PRELOAD = 518;

    public static int slideMotorPosition_THREE_CLOSE = 620; // 663

    public static int slideMotorPosition_THREE_FAR = 1850;

    public static double slideMotorPower_EXTENDING = 1;
    public static double slideMotorPower_RETRACTING = 1;
    public static double slideMotorPower_STILL = 0;

    public static double potentiometer_THREE_DEPOSIT = .98;
    public static double potentiometer_THREE_PRELOAD = 2.5;
    public static double potentiometer_MAX = 0.46;
    public static double potentiometer_AUTON = 2.5;
    public static double potentiometer_FAR = 1.44;
    public static double potentiometer_IF_LOWER_THAN = .44;
    public static double potentiometer_IF_HIGHER_THAN = 2.6;


    public static double liftMotorPower_MOVING = 0.1;

    public static double flipper_READY = .97;//0.825;
    public static double flipper_THREE_PRELOAD = .05;//0.37;
    public static double flipper_TWO_PRELOAD = .05;//0.2;
    public static double flipper_ONE_PRELOAD = .05;//0.085;
    public static double flipper_THREE_CLOSE = .05;//0.21;
    public static double flipper_THREE_FAR = .05;//0.18;
    public static double flipper_CUSTOM = .32;//0.18;

    public static int slideMotorTolerance = 8;
    public static int busyWait = 5; //ms
    public static int slideMotorExtensionThreshold = 150;
    public static double flipperOffset = .04;

    public static double driftOffset = 0.875;
}