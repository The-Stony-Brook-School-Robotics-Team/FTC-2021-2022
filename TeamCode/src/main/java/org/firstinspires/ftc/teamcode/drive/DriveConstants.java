package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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
@Config
public class DriveConstants {

    // MARK - RR Constants
    public static final double TICKS_PER_REV = 8192; // added
    public static final double MAX_RPM = 1150; // was 1100


    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 1.97;
    public static double GEAR_RATIO = 8.0 / 14;
    public static double TRACK_WIDTH = 13.25;

    public static double kV = 0.015; // Previous was 0.01
    public static double kA = 0.004; // Previous was 0.012
    public static double kStatic = 0.0134;

    public static double MAX_VEL = 100; // 80
    public static double MAX_ACCEL = 60;
    public static double MAX_ANG_VEL = 2; // measured with max ang velo tuner.
    public static double MAX_ANG_ACCEL = 2;


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

    // MARK - Destinations

    public static Pose2d A_StartPose = new Pose2d(0,0,0);
    public static Pose2d A_Carousel = new Pose2d(0,0,0);
    public static Pose2d A_SplineBegin = new Pose2d(0, 65, -Math.PI / 2);
    public static Pose2d A_SplineInt1 = new Pose2d(43.0, 48.0, -Math.PI * 3.0 / 4.0);
    public static Pose2d A_SplineInt2 = new Pose2d(65.0, 24.0, -Math.PI/1.0);
    public static Pose2d A_SplineEnd = new Pose2d(65.0, 12.0, -Math.PI);
    public static Pose2d A_DepositSlide = new Pose2d(0,0,0);
    public static Pose2d A_PickupObject = new Pose2d(0,0,0);


    // MARK - Trajectories

    public static Trajectory A_StartToCarousel;
    public static Trajectory A_CarouselToSplineBegin;
    public static Trajectory A_Spline;
    public static Trajectory A_GoToDeposit1;
    public static Trajectory A_GoToPickup1;
    public static Trajectory A_GoToDeposit2;
    public static Trajectory A_GoToPickup2;
    public static Trajectory A_GoToDeposit3;
    public static Trajectory A_GoToPickup3;
    public static Trajectory A_GoToDeposit4;
    public static Trajectory A_GoToPickup4;

    public static void instantiateTrajectories(SampleMecanumDrive drive) {
        // create all trajectories using the trajectory builder.
        A_StartToCarousel = drive.trajectoryBuilder(A_StartPose)
                .lineToSplineHeading(A_Carousel)
                .build();
        A_CarouselToSplineBegin = drive.trajectoryBuilder(A_Carousel)
                .lineToSplineHeading(A_SplineBegin)
                .build();
        A_Spline = drive.trajectoryBuilder(A_SplineBegin)
                .strafeLeft(24.0)
                .splineToSplineHeading(A_SplineInt1, -Math.PI / 4.0)
                .splineToSplineHeading(A_SplineInt2, -Math.PI / 2.0)
                .splineToSplineHeading(A_SplineEnd, -Math.PI / 2.0)
                .build();

        A_GoToDeposit1 = drive.trajectoryBuilder(A_SplineEnd)
                .lineToSplineHeading(A_DepositSlide)
                .build();

        A_GoToPickup1 = drive.trajectoryBuilder(A_DepositSlide)
                .lineToSplineHeading(A_PickupObject)
                .build();

        A_GoToDeposit2 = drive.trajectoryBuilder(A_SplineEnd)
                .lineToSplineHeading(A_DepositSlide)
                .build();

        A_GoToPickup2 = drive.trajectoryBuilder(A_DepositSlide)
                .lineToSplineHeading(A_PickupObject)
                .build();
        A_GoToDeposit3 = drive.trajectoryBuilder(A_SplineEnd)
                .lineToSplineHeading(A_DepositSlide)
                .build();

        A_GoToPickup3 = drive.trajectoryBuilder(A_DepositSlide)
                .lineToSplineHeading(A_PickupObject)
                .build();

        A_GoToDeposit4 = drive.trajectoryBuilder(A_SplineEnd)
                .lineToSplineHeading(A_DepositSlide)
                .build();

        A_GoToPickup4 = drive.trajectoryBuilder(A_DepositSlide)
                .lineToSplineHeading(A_PickupObject)
                .build();

    }


















}
