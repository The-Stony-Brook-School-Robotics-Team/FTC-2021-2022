package org.firstinspires.ftc.teamcode.common.tuning.timeout;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class ManualPointTest extends LinearOpMode {

    public static double DISTANCE = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            Trajectory trajectoryForward = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(DISTANCE)
                    .build();


            drive.followTrajectory(trajectoryForward);
            Trajectory trajectoryBackward = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(DISTANCE)
                    .build();
            drive.followTrajectory(trajectoryBackward);
        }
    }
}