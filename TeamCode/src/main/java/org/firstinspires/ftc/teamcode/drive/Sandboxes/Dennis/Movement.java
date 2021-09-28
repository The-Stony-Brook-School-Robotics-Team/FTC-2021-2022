package org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.jetbrains.annotations.NotNull;

public class Movement {

    public static Pose2d forward(@NotNull Pose2d currentPos, @NotNull SampleMecanumDrive drive, @NotNull int inches) {
        Trajectory traj =  drive.trajectoryBuilder(currentPos)
                .forward(inches)
                .build();
        drive.followTrajectory(traj);
        drive.update();
        return drive.getPoseEstimate();
    }

    public static Pose2d strafeLeft(@NotNull Pose2d currentPos, @NotNull SampleMecanumDrive drive, @NotNull int inches) {
        Trajectory traj =  drive.trajectoryBuilder(currentPos)
                .strafeLeft(inches)
                .build();
        drive.followTrajectory(traj);
        drive.update();
        return drive.getPoseEstimate();
    }

    public static Pose2d strafeRight(@NotNull Pose2d currentPos, @NotNull SampleMecanumDrive drive, @NotNull int inches) {
        Trajectory traj =  drive.trajectoryBuilder(currentPos)
                .strafeRight(inches)
                .build();
        drive.followTrajectory(traj);
        drive.update();
        return drive.getPoseEstimate();
    }

    public static Pose2d back(@NotNull Pose2d currentPos, @NotNull SampleMecanumDrive drive, @NotNull int inches) {
        Trajectory traj =  drive.trajectoryBuilder(currentPos)
                .back(inches)
                .build();
        drive.followTrajectory(traj);
        drive.update();
        return drive.getPoseEstimate();
    }





}
