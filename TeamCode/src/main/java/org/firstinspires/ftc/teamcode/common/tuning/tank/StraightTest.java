package org.firstinspires.ftc.teamcode.common.tuning.tank;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Autonomous(group = "drive", name = "T - TANKStraightTest")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 30; // in

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Trajectory traj1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(DISTANCE)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(-DISTANCE)
                .build();

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {
            drive.followTrajectory(traj1);
            Pose2d p1 = drive.getPoseEstimate();
            telemetry.addData("finalX", p1.getX());
            telemetry.addData("finalY", p1.getY());
            telemetry.addData("finalHeading", p1.getHeading());
            telemetry.update();
            drive.followTrajectory(traj2);
            Pose2d p2 = drive.getPoseEstimate();
            telemetry.addData("finalX", p2.getX());
            telemetry.addData("finalY", p2.getY());
            telemetry.addData("finalHeading", p2.getHeading());
            telemetry.update();

        }

    }
}
