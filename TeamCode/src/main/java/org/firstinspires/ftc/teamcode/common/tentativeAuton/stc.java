package org.firstinspires.ftc.teamcode.common.tentativeAuton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;

@TeleOp(name = "----stc inspire award celebration", group = "STC")
public class stc extends LinearOpMode {
    public static double MAX_VELOCITY = 30;
    public static double MAX_ACCELERATION = 10;

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

        TrajectoryVelocityConstraint tvc = SampleMecanumDrive.getVelocityConstraint(MAX_VELOCITY,3,9.5);
        TrajectoryAccelerationConstraint tac = SampleMecanumDrive.getAccelerationConstraint(MAX_ACCELERATION);

        RRDrive.followTrajectory(
                RRDrive.trajectoryBuilder(
                        RRDrive.getPoseEstimate(),tvc,tac)
                        .splineToSplineHeading(new Pose2d(31.51588944736609, 82.78047852985107, 1.5396784188674317), Math.toRadians(90.0))
                        .splineToSplineHeading(
                                new Pose2d(14.448214975643213, 99.94967611888816, 3.1206004029256498),
                                Math.toRadians(180.0)
                        )
                        .splineToSplineHeading(
                                new Pose2d(-2.7420649476401002, 82.6480860639469, 4.667356119961884),
                                Math.toRadians(270.0)
                        )
                        .splineToSplineHeading(new Pose2d(14.0, 65.5, 0.0), Math.toRadians(0.0))
                        .splineToSplineHeading(new Pose2d(31.51588944736609, 82.78047852985107, 1.5396784188674317), Math.toRadians(90.0))
                        .splineToSplineHeading(
                                new Pose2d(14.448214975643213, 99.94967611888816, 3.1206004029256498),
                                Math.toRadians(180.0)
                        )
                        .splineToSplineHeading(
                                new Pose2d(-2.7420649476401002, 82.6480860639469, 4.667356119961884),
                                Math.toRadians(270.0)
                        )
                        .splineToSplineHeading(new Pose2d(14.0, 65.5, 0.0), Math.toRadians(0.0))
                        .splineToSplineHeading(new Pose2d(31.51588944736609, 82.78047852985107, 1.5396784188674317), Math.toRadians(90.0))
                        .splineToSplineHeading(
                                new Pose2d(14.448214975643213, 99.94967611888816, 3.1206004029256498),
                                Math.toRadians(180.0)
                        )
                        .splineToSplineHeading(
                                new  Pose2d(-2.7420649476401002, 82.6480860639469, 4.667356119961884),
                                Math.toRadians(270.0)
                        )
                        .splineToSplineHeading(new Pose2d(14.0, 65.5, 0.0), Math.toRadians(0.0))
                        .splineToSplineHeading(new Pose2d(31.51588944736609, 82.78047852985107, 1.5396784188674317), Math.toRadians(90.0))
                        .splineToSplineHeading(
                                new Pose2d(14.448214975643213, 99.94967611888816, 3.1206004029256498),
                                Math.toRadians(180.0)
                        )
                        .splineToSplineHeading(
                                new Pose2d(-2.7420649476401002, 82.6480860639469, 4.667356119961884),
                                Math.toRadians(270.0)
                        )
                        .splineToSplineHeading(new Pose2d(14.0, 65.5, 0.0), Math.toRadians(0.0))
                        .build()
        );

        updatePose.interrupt();
        brain.majorState.set(AutonomousBrain.MajorAutonomousState.FINISHED);
        brain.minorState.set(AutonomousBrain.MinorAutonomousState.STOPPED);
        requestOpModeStop();
        stop();
    }
}
