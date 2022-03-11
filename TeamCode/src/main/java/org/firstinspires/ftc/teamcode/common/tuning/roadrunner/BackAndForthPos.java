package org.firstinspires.ftc.teamcode.common.tuning.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */

//@Autonomous(name = "T - BackAndForthPos", group = "drive")
public class BackAndForthPos extends LinearOpMode {

    public static double DISTANCE = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        Pose2d ini = drive.getPoseEstimate();
        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            Trajectory trajectoryForward = drive.trajectoryBuilder(ini)
                    .lineToSplineHeading(new Pose2d(ini.getX()+DISTANCE,ini.getY(),ini.getHeading()))
                    .build();


            drive.followTrajectory(trajectoryForward);
            Trajectory trajectoryBackward = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(ini)
                    .build();
            drive.followTrajectory(trajectoryBackward);
        }
//        T265Controller.shutDown();
    }
}