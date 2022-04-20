package org.firstinspires.ftc.teamcode.common.official.auton.v1.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveConstants {

    // robot
    public static final double TICKS_PER_REV = 537.7;
    public static double WHEEL_RADIUS = 1.889765;
    public static double GEAR_RATIO = 1.5; //2.15

    // pid
    public static int positionTolerance = 20;

}
