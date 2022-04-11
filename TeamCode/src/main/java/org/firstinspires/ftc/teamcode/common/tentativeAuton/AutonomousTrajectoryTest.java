package org.firstinspires.ftc.teamcode.common.tentativeAuton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "AutonomousTrajectoryTest")
public class AutonomousTrajectoryTest extends LinearOpMode {
    public static Vector2d PICK_UP_POSE = new Vector2d(63.0, 70.0);
    public static Pose2d DEPOSIT_POSE = new Pose2d(5.58,64.47, -Math.toRadians(30));

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
                        .lineToLinearHeading(DEPOSIT_POSE)
                        .build()
        );

        while (opModeIsActive()) {
            RRDrive.followTrajectory(
                    RRDrive.trajectoryBuilder(
                            RRDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(15.0, 67.0, 0.0))
                            .splineToConstantHeading(PICK_UP_POSE, 0.0)
                            .build()
            );

            RRDrive.followTrajectory(
                    RRDrive.trajectoryBuilder(
                            RRDrive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(40.0, 69.0, 0.0))
                            .splineToConstantHeading(new Vector2d(18.0, 66.0), Math.toRadians(-160.0))
                            .splineToSplineHeading(DEPOSIT_POSE, Math.toRadians(-175.0))
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
