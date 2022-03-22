package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class NewAutonomousMecanumDrive extends SampleMecanumDrive {

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(30, 0.0, 7);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(30, 0, 1);

    public NewAutonomousMecanumDrive(HardwareMap hardwareMap) {
        super(hardwareMap, TRANSLATIONAL_PID, HEADING_PID);
    }
}