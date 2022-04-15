package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstantsTank {

    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;

    public static boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(5, 0, 0,
            0);

    public static double WHEEL_RADIUS = 1.889765;
    //24/16 ?
    public static double GEAR_RATIO = 1.5;
    public static double TRACK_WIDTH = 10.3;

    public static double kV = 0.0138;
    public static double kA = 0.0044;
    public static double kStatic = 0.012;

    public static double MAX_VEL = 55;
    public static double MAX_ACCEL = 55;
    public static double MAX_ANG_VEL = Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

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