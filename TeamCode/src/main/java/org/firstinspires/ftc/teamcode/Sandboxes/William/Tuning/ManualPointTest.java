package org.firstinspires.ftc.teamcode.sandboxes.William.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sandboxes.William.Util.CustomizedMecanumDrive;

@Config
@Autonomous(group = "drive", name="***** Manual Point Test")
public class ManualPointTest extends LinearOpMode {

    public static double DISTANCE = 48;

    public static double customizedMecanumDriveDistanceError_IN = Double.MAX_VALUE;
    public static double customizedMecanumDriveHeadingError_DE = 360;
    public static double customizedMecanumDriveTimeoutValue_SE = 10.0;

    @Override
    public void runOpMode() throws InterruptedException {
        CustomizedMecanumDrive customizedMecanumDrive = new CustomizedMecanumDrive(hardwareMap);


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            //Go to target point.
            customizedMecanumDrive.setAcceptableDistanceError_IN(customizedMecanumDriveDistanceError_IN);
            customizedMecanumDrive.setAcceptableHeadingError_DE(customizedMecanumDriveHeadingError_DE);
            customizedMecanumDrive.setTimeoutValue_SE(customizedMecanumDriveTimeoutValue_SE);
            Trajectory trajectoryForward = customizedMecanumDrive.trajectoryBuilder(customizedMecanumDrive.getPoseEstimate())
                    .forward(DISTANCE)
                    .build();
            customizedMecanumDrive.followTrajectory(trajectoryForward);

            //Return to start point.
            customizedMecanumDrive.resetFollower();
            Trajectory trajectoryBackward = customizedMecanumDrive.trajectoryBuilder(customizedMecanumDrive.getPoseEstimate())
                    .back(DISTANCE)
                    .build();
            customizedMecanumDrive.followTrajectory(trajectoryBackward);
        }
    }
}