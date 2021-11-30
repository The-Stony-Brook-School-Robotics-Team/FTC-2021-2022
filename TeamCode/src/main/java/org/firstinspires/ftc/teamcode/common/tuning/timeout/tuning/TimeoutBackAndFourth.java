package org.firstinspires.ftc.teamcode.common.tuning.timeout.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.timeout.CustomTimeoutTuningDrive;

@Config
@Autonomous(group="drive", name="T - Timeout Back And Fourth")
public class TimeoutBackAndFourth extends LinearOpMode {

    public static double DISTANCE = 48;
    public static double TIMEOUT = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        CustomTimeoutTuningDrive drive = new CustomTimeoutTuningDrive(hardwareMap, TIMEOUT);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(DISTANCE, 0, 0))
                .build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryForward.end())
                .lineToSplineHeading(new Pose2d(0, 0, 0))
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive.followTrajectory(trajectoryForward);
            drive.followTrajectory(trajectoryBackward);
        }
        //T265Controller.shutDown();
    }
}