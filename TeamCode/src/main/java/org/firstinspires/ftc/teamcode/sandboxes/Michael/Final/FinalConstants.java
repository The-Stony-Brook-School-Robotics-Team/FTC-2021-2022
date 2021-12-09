package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class FinalConstants {

    /*
     * Constants shared between multiple drive types.
     *
     * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
     * fields may also be edited through the dashboard (connect to the robot's WiFi network and
     * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
     * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
     *
     * These are not the only parameters; some are located in the localizer classes, drive base classes,
     * and op modes themselves.
     */

        //TODO --Localizer Constants--
        //Odometry Wheels
        public static double ODOMETRY_TICKS_PER_REV = 8192;
        public static double ODOMETRY_WHEEL_RADIUS = 1; // in
        public static double ODOMETRY_GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

        public static double ODOMETRY_LATERAL_DISTANCE = 12.3; // in; distance between the left and right wheels
        public static double ODOMETRY_FORWARD_OFFSET = -7.5; // in; offset of the lateral wheel

        //TODO --Drive Constants--
        //Mecanum Wheels
        public static final double MECANUM_TICKS_PER_REV = 8192; // added
        public static final double MAX_RPM = 1150; // rotations per minute; was 1100


        public static final boolean RUN_USING_ENCODER = false;
        public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
                getMotorVelocityF(MAX_RPM / 60 * MECANUM_TICKS_PER_REV));

        public static double MECANUM_WHEEL_RADIUS = 1.97;
        public static double MECANUM_GEAR_RATIO = 8.0 / 14;
        public static double TRACK_WIDTH = 13.25;

        public static double kV = 0.015; // Previous was 0.01
        public static double kA = 0.004; // Previous was 0.012
        public static double kStatic = 0.0134;

        public static double MAX_VEL = 100; //was 80
        public static double MAX_ACCEL = 60;
        public static double MAX_ANG_VEL = 2; // measured with max ang velo tuner.
        public static double MAX_ANG_ACCEL = 2;


        public static double encoderTicksToInches(double ticks) {
            return MECANUM_WHEEL_RADIUS * 2 * Math.PI * MECANUM_GEAR_RATIO * ticks / MECANUM_TICKS_PER_REV;
        }

        public static double rpmToVelocity(double rpm) {
            return rpm * MECANUM_GEAR_RATIO * 2 * Math.PI * MECANUM_WHEEL_RADIUS / 60.0;
        }

        public static double getMotorVelocityF(double ticksPerSecond) {
            // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
            return 32767 / ticksPerSecond;
        }

        //TODO --SampleMecanumDrive Constants--
        public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(20, 0, 1);
        public static PIDCoefficients HEADING_PID = new PIDCoefficients(50, 0, 1); // tasty

        public static double LATERAL_MULTIPLIER = 1;

        public static double VX_WEIGHT = 1;
        public static double VY_WEIGHT = 1;
        public static double OMEGA_WEIGHT = 1;
}
