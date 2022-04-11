package org.firstinspires.ftc.teamcode.common.tentativeAuton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "AutonomousTrajectoryTest")
public class AutonomousTrajectoryTest extends LinearOpMode {
    public static double PICK_UP_POSE_X = 53.0;
    public static double PICK_UP_POSE_Y = 70.0;
    public static double PICK_UP_FIX_HEADING_X = 15.0;
    public static double PICK_UP_FIX_HEADING_Y = 67.0;
    public static double DEPOSIT_FIX_HEADING_X = 40.0;
    public static double DEPOSIT_FIX_HEADING_Y = 69.0;
    public static double DEPOSIT_PASS_PIPE_X = 18.0;
    public static double DEPOSIT_PASS_PIPE_Y = 66.0;

    AutonomousBrain brain;
    SampleMecanumDrive RRDrive;

    Thread updatePose = new Thread(() -> {
        while (opModeIsActive() && !Thread.interrupted()) {
            try {
                if (RRDrive.isRunningFollowTrajectory)
                    Thread.sleep(20);

                brain.RRctrl.getDrive().update();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        brain.RRctrl.stopRobot();
    });

    @Override
    public void runOpMode() {
        brain = new AutonomousBrain(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);
        updatePose.start();
        RRDrive = brain.RRctrl.getDrive();

        waitForStart();
        RRDrive.followTrajectory(
                RRDrive.trajectoryBuilder(
                        RRDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(7.15, 63.0, Math.toRadians(33.0)))
                        .build()
        );

        while (opModeIsActive()) {
            RRDrive.followTrajectory(
                    RRDrive.trajectoryBuilder(
                            RRDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(PICK_UP_FIX_HEADING_X, PICK_UP_FIX_HEADING_Y, 0.0))
                            .splineToConstantHeading(new Vector2d(PICK_UP_POSE_X, PICK_UP_POSE_Y), 0.0)
                            .build()
            );

            RRDrive.followTrajectory(
                    RRDrive.trajectoryBuilder(
                            RRDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(DEPOSIT_FIX_HEADING_X, DEPOSIT_FIX_HEADING_Y, 0.0))
                            .splineToConstantHeading(new Vector2d(DEPOSIT_PASS_PIPE_X, DEPOSIT_PASS_PIPE_Y), Math.toRadians(-160.0))
                            .splineToSplineHeading(new Pose2d(7.15, 63.0, Math.toRadians(33.0)), Math.toRadians(-175.0))
                            .build()
            );
        }

        updatePose.interrupt();
        brain.majorState.set(AutonomousBrain.MajorAutonomousState.FINISHED);
        brain.minorState.set(AutonomousBrain.MinorAutonomousState.STOPPED);
        requestOpModeStop();
        stop();
    }
}
