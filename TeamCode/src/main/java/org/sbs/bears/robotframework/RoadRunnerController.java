package org.sbs.bears.robotframework;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sandboxes.Marc.AutonSimulator;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;


public class RoadRunnerController {

    boolean isRunningInterruptibleTraj = false;

    SampleMecanumDrive drive;
    FtcDashboard dashboard;
    TrajectorySequenceRunner runner;
    Object internalMutex = new Object();

    public RoadRunnerController(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.dashboard = FtcDashboard.getInstance();
        this.runner = drive.trajectorySequenceRunner;
    }
    public void shutDown()
    {
        // TODO implement ShutDown on RR Ctrl
    }
    public void forwardHaltableTrajectory(double distMax, double brakingDist, double brakeVel, double brakeDecel, Boolean signal, Object mutex)
    {
        boolean isRunning = true;
        startInterruptibleTrajVar();
        boolean isForwarding = false;
        boolean isBraking = false;
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(brakeVel, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(brakeDecel);


        new Thread(()->{Boolean tmpIsRunning = Boolean.getBoolean("true");
            while(isRunningInterruptibleTraj) {
                synchronized (mutex) {
                    if(signal) {
                        runner.cancelTraj();
                        return;
                    }
                }
            }
        }).start();

        double iniX = drive.getPoseEstimate().getX();
        double iniTime = NanoClock.system().seconds();
        Trajectory trajForward = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(distMax,0,0))
                .build();
        isForwarding = true;
        drive.followTrajectory(trajForward); // interruptible
        isForwarding = false;
        haltInterruptibleTrajVar();
        isBraking = true;
        double currentX = drive.getPoseEstimate().getX();
        Trajectory trajBrake = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(brakingDist+currentX-iniX,velocityConstraint,accelerationConstraint)
                .build();
        drive.followTrajectoryTime(trajBrake,iniTime);
        isRunning = false;
        isBraking = false;
    }

    private void startInterruptibleTrajVar()
    {
        isRunningInterruptibleTraj = true;
    }
    private void haltInterruptibleTrajVar()
    {
        isRunningInterruptibleTraj = false;
    }



}
