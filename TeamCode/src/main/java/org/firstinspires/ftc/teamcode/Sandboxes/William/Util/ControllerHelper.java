package org.firstinspires.ftc.teamcode.Sandboxes.William.Util;

public class ControllerHelper {
    public static double COUNTS_PER_INCH = 1304.777; //307.699557

    public static final String LEFT_FRONT_MOTOR_NAME = "lf";
    public static final String RIGHT_FRONT_MOTOR_NAME = "leftodom";
    public static final String LEFT_BACK_MOTOR_NAME = "backodom";
    public static final String RIGHT_BACK_MOTOR_NAME = "rightodom";

    public static final String LEFT_ODOMETRY_NAME = "leftodom";
    public static final String RIGHT_ODOMETRY_NAME = "rightodom";
    public static final String BACK_ODOMETRY_NAME = "backodom";

    public static final String LEFT_FRONT_ENCODER_NAME = "lfencoder";
    public static final String RIGHT_FRONT_ENCODER_NAME = "rfencoder";
    public static final String LEFT_BACK_ENCODER_NAME = "lbencoder";
    public static final String RIGHT_BACK_ENCODER_NAME = "rbencoder";

    public static final int THREAD_SLEEP_DELAY = 75;

    public static final double ROBOT_ENCODER_WHEEL_DISTANCE = 12.75;
    public static final double HORIZONTAL_ENCODER_TICK_PER_DEGREE_OFFSET = -8.7;

    private static final double TRACKWIDTH = 12.75;         //Actual measurement: 12.00
    private static final double CENTER_WHEEL_OFFSET = -8.7; //Actual measurement: -7.5

    private static final double WHEEL_DIAMETER = 2.0;
    private static final double TICKS_PER_REV = 8192;       //Actual measurement:

    public static double WHEEL_RADIUS = 1.97;
    public static double GEAR_RATIO = 8.0 / 14;
}
