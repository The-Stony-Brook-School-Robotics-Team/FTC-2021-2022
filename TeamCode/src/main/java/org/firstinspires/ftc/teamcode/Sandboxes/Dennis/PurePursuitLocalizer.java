package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.BearsUtil.T265Controller;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;


@Config
public class PurePursuitLocalizer implements Localizer {
    private static final double TRACKWIDTH = 12.75;
    private static final double CENTER_WHEEL_OFFSET = -8.7;

    private static final double WHEEL_DIAMETER = 2.0;
    private static final double TICKS_PER_REV = 8192;

    private static final double TICKS_TO_INCHES = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;


    private Encoder leftEncoder, rightEncoder, frontEncoder;
    private MotorEx lf, rf, lb, rb;

    HolonomicOdometry holOdom;
    OdometrySubsystem odometry;

    private MotorEx encoderLeft, encoderRight, encoderPerp;
    private MecanumDriveKinematics kinematics;

    public PurePursuitLocalizer(HardwareMap hardwareMap) {
        // Mapping
        lf = new MotorEx(hardwareMap, "lf");
        rf = new MotorEx(hardwareMap, "leftodom");
        lb = new MotorEx(hardwareMap, "backodom");
        rb = new MotorEx(hardwareMap, "rightodom");

       // leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftodom"));
       // rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightodom"));
        //frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backodom"));

        // Mapping End

        // Pure Pursuit Start -------------
        holOdom = new HolonomicOdometry(
                () -> (leftEncoder.getCurrentPosition() * TICKS_TO_INCHES),
                () -> -(rightEncoder.getCurrentPosition() * TICKS_TO_INCHES),
                () -> (frontEncoder.getCurrentPosition() * TICKS_TO_INCHES),
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        encoderLeft = new MotorEx(hardwareMap, "leftodom");
        encoderRight = new MotorEx(hardwareMap, "rightodom");
        encoderPerp = new MotorEx(hardwareMap, "backodom");

        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);


        encoderLeft.resetEncoder();
        encoderRight.resetEncoder();
        encoderPerp.resetEncoder();
        // Pure Pursuit End -----------

        // Kinematics
        Translation2d m_frontLeftLocation = new Translation2d(7, 6.5);
        Translation2d m_frontRightLocation = new Translation2d(7, -6.5);
        Translation2d m_backLeftLocation = new Translation2d(-7, 6.5);
        Translation2d m_backRightLocation = new Translation2d(-7, -6.5);

        kinematics = new MecanumDriveKinematics(
                m_frontLeftLocation, m_frontRightLocation,
                m_backLeftLocation, m_backRightLocation
        );
        // Kinematics End

        // Set the Sub System
        odometry = new OdometrySubsystem(holOdom);
    }

    public Pose2d getPoseEstimate() {
        Pose2d pos = new Pose2d(odometry.getPose().getX(), -odometry.getPose().getY(), -odometry.getPose().getHeading());
        return pos;
    }

    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        holOdom.updatePose(T265Controller.reverseConvert265PosToRR(pose2d));
    }

    public Pose2d getPoseVelocity() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(lf.getVelocity(),rf.getVelocity(),lb.getVelocity(),rb.getVelocity());
        ChassisSpeeds sped = kinematics.toChassisSpeeds(wheelSpeeds);
        Pose2d vel = new Pose2d(sped.vxMetersPerSecond,sped.vyMetersPerSecond, sped.omegaRadiansPerSecond);
        return vel;
    }

    public void update() {
        holOdom.updatePose();
    }
}

