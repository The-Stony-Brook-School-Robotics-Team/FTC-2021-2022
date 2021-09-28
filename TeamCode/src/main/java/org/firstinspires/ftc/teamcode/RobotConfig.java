package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConfig {
    // Odom Offsets
    public static double WHEEL_RADIUS = 1.97;
    public static double GEAR_RATIO = 8.0 / 14;
    public static double TRACK_WIDTH = 12.75;
    public static final double TICKS_PER_REV = 145.6;
    public static double WHEEL_DIAMETER = WHEEL_RADIUS * 2;
    public static final double TICKS_TO_INCHES  = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    public static final double CENTER_WHEEL_OFFSET = -8.7;
}
