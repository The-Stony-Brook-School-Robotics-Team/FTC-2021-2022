package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.BearsUtil.T265Controller;

public class PurePursuitLocalizer implements Localizer {
    private static final double TRACKWIDTH = 12.75;
    private static final double CENTER_WHEEL_OFFSET = -8.7;
    private static final double WHEEL_DIAMETER = 2.0;
    private static final double TICKS_PER_REV = 8192;
    private static final double TICKS_TO_INCHES = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    private HolonomicOdometry holOdom;
    private MotorEx lf, rf, lb, rb;
    private MecanumDrive drive;
    private OdometrySubsystem odometry;
    private MotorEx encoderLeft, encoderRight, encoderPerp;
    private MecanumDriveKinematics kinematics;



    public PurePursuitLocalizer(HardwareMap hardwareMap) {
        lf = new MotorEx(hardwareMap, "lf");
        rf = new MotorEx(hardwareMap, "leftodom");
        lb = new MotorEx(hardwareMap, "backodom");
        rb = new MotorEx(hardwareMap, "rightodom");

        encoderLeft = new MotorEx(hardwareMap, "leftodom");
        encoderRight = new MotorEx(hardwareMap, "rightodom");
        encoderPerp = new MotorEx(hardwareMap, "backodom");

        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        encoderLeft.resetEncoder();
        encoderRight.resetEncoder();
        encoderPerp.resetEncoder();

        //robotDrive = new MecanumDrive(lf, rf, lb, rb);
        //dashboard = FtcDashboard.getInstance();

        holOdom = new HolonomicOdometry(
                () -> encoderLeft.getCurrentPosition() * TICKS_TO_INCHES,
                () -> -(encoderRight.getCurrentPosition() * TICKS_TO_INCHES),
                () -> (encoderPerp.getCurrentPosition() * TICKS_TO_INCHES),
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        odometry = new OdometrySubsystem(holOdom);


        Translation2d m_frontLeftLocation = new Translation2d(7, 6.5);
        Translation2d m_frontRightLocation = new Translation2d(7, -6.5);
        Translation2d m_backLeftLocation = new Translation2d(-7, 6.5);
        Translation2d m_backRightLocation = new Translation2d(-7, -6.5);




        // LF RF LB RB
        kinematics = new MecanumDriveKinematics(
                m_frontLeftLocation, m_frontRightLocation,
                m_backLeftLocation, m_backRightLocation
        );
    }

    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        com.arcrobotics.ftclib.geometry.Pose2d posPP = odometry.getPose();
        Pose2d pos = new Pose2d(posPP.getX(),posPP.getY(),posPP.getHeading());
        return pos;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        holOdom.updatePose(T265Controller.reverseConvert265PosToRR(pose2d));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {  // LF RF LB RB
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(lf.getVelocity(),rf.getVelocity(),lb.getVelocity(),rb.getVelocity());
        ChassisSpeeds sped = kinematics.toChassisSpeeds(wheelSpeeds);
        //com.arcrobotics.ftclib.geometry.Pose2d velPP = odometry.
       Pose2d vel = new Pose2d(sped.vxMetersPerSecond,sped.vyMetersPerSecond, sped.omegaRadiansPerSecond);
        return vel;
    }

    @Override
    public void update() {
        holOdom.updatePose();
    }
}
