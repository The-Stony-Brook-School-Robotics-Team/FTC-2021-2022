package org.firstinspires.ftc.teamcode.common.tuning.timeout;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;

public class InfiniteDrive implements Runnable {
    Trajectory trajectoryForward;
    SampleMecanumDrive drive;

    @Override
    public void run() {
        drive.followTrajectory(trajectoryForward);
    }
}
