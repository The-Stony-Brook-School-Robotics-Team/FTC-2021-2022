package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
public class DriveConstants {


    public static final double TICKS_PER_REV = 145.1;
    public static final double MAX_RPM = 1150;


    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(30, 0, 4,
            0.2);


    public static double WHEEL_RADIUS = 1.97; // in
    public static double GEAR_RATIO = 14.0 / 8; // output (wheel) speed / input (motor) speed 8/14
    public static double TRACK_WIDTH = 13.25; // in


    public static double kV = .007;
    public static double kA = 0.001;
    public static double kStatic = 0;


    public static double MAX_VEL = 150;
    public static double MAX_ACCEL = 30;
    public static double MAX_ANG_VEL = Math.toRadians(60);
    public static double MAX_ANG_ACCEL = Math.toRadians(60);


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }
}