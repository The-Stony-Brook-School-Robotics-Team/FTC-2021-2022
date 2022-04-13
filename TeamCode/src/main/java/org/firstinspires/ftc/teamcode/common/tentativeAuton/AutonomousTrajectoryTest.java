package org.firstinspires.ftc.teamcode.common.tentativeAuton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "------------AutonomousTrajectoryTest")
public class AutonomousTrajectoryTest extends LinearOpMode {
    public static double PICK_UP_POSE_X = 55.0;
    public static double PICK_UP_POSE_Y = 66.0;
    public static double DEPOSIT_POSE_X = 7.15;
    public static double DEPOSIT_POSE_Y = 61.0;
    public static double PICK_UP_FIX_HEADING_X = 14.0;
    public static double PICK_UP_FIX_HEADING_Y = 66.5;
    public static double DEPOSIT_FIX_HEADING_X = 40.0;
    public static double DEPOSIT_FIX_HEADING_Y = 67.0;
    public static double DEPOSIT_PASS_PIPE_X = 18.0;
    public static double DEPOSIT_PASS_PIPE_Y = 66.0;
    public static int SLEEP_TIME = 500;

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
    public void runOpMode() throws InterruptedException {
        brain = new AutonomousBrain(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);
        updatePose.start();
        RRDrive = brain.RRctrl.getDrive();

        waitForStart();
        RRDrive.followTrajectory(
                RRDrive.trajectoryBuilder(
                        RRDrive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(DEPOSIT_POSE_X, DEPOSIT_POSE_Y, Math.toRadians(-33.0)))
                        .build()
        );

        while (opModeIsActive()) {
            Thread.sleep(SLEEP_TIME);
            RRDrive.followTrajectory(
                    RRDrive.trajectoryBuilder(
                            RRDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(PICK_UP_FIX_HEADING_X, PICK_UP_FIX_HEADING_Y, 0.0))
                            .splineToConstantHeading(new Vector2d(PICK_UP_POSE_X, PICK_UP_POSE_Y), 0.0)
                            .addSpatialMarker(new Vector2d(40.0,70.0),()-> RRDrive.setPoseEstimate(new Pose2d(RRDrive.getPoseEstimate().getX(),65.5,RRDrive.getPoseEstimate().getHeading())))
                            .build()
            );
            Thread.sleep(SLEEP_TIME);
            RRDrive.followTrajectory(
                    RRDrive.trajectoryBuilder(
                            RRDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(DEPOSIT_FIX_HEADING_X, DEPOSIT_FIX_HEADING_Y, 0.0))
                            .splineToConstantHeading(new Vector2d(DEPOSIT_PASS_PIPE_X, DEPOSIT_PASS_PIPE_Y), Math.toRadians(-160.0))
                            .splineToSplineHeading(new Pose2d(DEPOSIT_POSE_X, DEPOSIT_POSE_Y, Math.toRadians(-33.0)), Math.toRadians(-175.0))
                            .addSpatialMarker(new Vector2d(35.0,70.0),()-> RRDrive.setPoseEstimate(new Pose2d(RRDrive.getPoseEstimate().getX(),65.5,RRDrive.getPoseEstimate().getHeading())))
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
