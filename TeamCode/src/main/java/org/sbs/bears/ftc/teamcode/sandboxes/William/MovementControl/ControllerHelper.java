package org.firstinspires.ftc.teamcode.sandboxes.William.MovementControl;

public class ControllerHelper {
    public static double COUNTS_PER_INCH = 1304.777; //307.699557

    public static final String leftFrontMotorName = "lf";
    public static final String rightFrontMotorName = "leftodom";
    public static final String leftBackMotorName = "backodom";
    public static final String rightBackMotorName = "rightodom";

    public static final String leftOdometryName = "leftodom";
    public static final String rightOdometryName = "rightodom";
    public static final String backOdometryName = "backodom";

    public static final String leftFrontEncoderName = "lfencoder";
    public static final String rightFrontEncoderName = "rfencoder";
    public static final String leftBackEncoderName = "lbencoder";
    public static final String rightBackEncoderName = "rbencoder";

    public static final int THREAD_SLEEP_DELAY = 75;

    public static double robotEncoderWheelDistance;
    public static double horizontalEncoderTickPerDegreeOffset;
}
