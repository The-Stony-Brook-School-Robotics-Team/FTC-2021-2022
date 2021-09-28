package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstantsNoOdom {
    public static final double TICKS_PER_REV = 145.6; // added
    public static final double MAX_RPM = 847; // was 1100

    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 1.97;
    public static double GEAR_RATIO = 8.0 / 14;
    public static double TRACK_WIDTH = 12.75;

    public static double kV = 0.0133; // Previous was 0.01
    public static double kA = 0.0055; // Previous was 0.012
    public static double kStatic = 0.01;

    public static double MAX_VEL = 80; // 80
    public static double MAX_ACCEL = 40;
    public static double MAX_ANG_VEL = 50; // measured with max ang velo tuner.
    public static double MAX_ANG_ACCEL = Math.toRadians(60);


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
