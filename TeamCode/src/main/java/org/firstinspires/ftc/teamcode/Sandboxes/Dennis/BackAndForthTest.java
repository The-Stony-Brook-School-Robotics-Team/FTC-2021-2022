package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveOdom;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrivePurePursuit;


@Config
@Autonomous(group = "drive", name="PP BackAndFourth")
public class BackAndForthTest extends LinearOpMode {

    public static double DISTANCE = 24;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrivePurePursuit drive = new SampleMecanumDrivePurePursuit(hardwareMap);



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