package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstantsMain {

    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 435;

    public static boolean RUN_USING_ENCODER = false; //true for odom + drive encoders
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            0);

    public static double WHEEL_RADIUS = 1.47368; // in
    public static double GEAR_RATIO = 1.5059055118; // 14.0 / 8; // output (wheel) speed / input (motor) speed 8/14
    public static double TRACK_WIDTH = 9.5; // in 10

    public static double kV = 0.0115;
    public static double kA = 0.0064;
    public static double kStatic = 0.03;

    public static double MAX_VEL = 80;
    public static double MAX_ACCEL = 45;
    public static double MAX_ANG_VEL = 3;
    public static double MAX_ANG_ACCEL = 2;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }
    public static double inchesToEncoderTicks(double inches) {
        return inches * TICKS_PER_REV / WHEEL_RADIUS / 2 / Math.PI / GEAR_RATIO;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}