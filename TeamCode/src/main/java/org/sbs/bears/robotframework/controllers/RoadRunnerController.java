package org.sbs.bears.robotframework.controllers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

import java.util.Vector;


public class RoadRunnerController {

    boolean isRunningInterruptibleTraj = false;

    protected SampleMecanumDrive drive;
    protected FtcDashboard dashboard;
    protected TrajectorySequenceRunner runner;
    Object internalMutex = new Object();

    public RoadRunnerController(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.dashboard = FtcDashboard.getInstance();
        this.runner = drive.trajectorySequenceRunner;
    }
    public void forward(Pose2d iniPos, double dist)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(iniPos)
                        .forward(dist)
                        .build()
        );
    }
    public Pose2d getPos()
    {
        return drive.getPoseEstimate();
    }
    public void setPos(Pose2d newPos)
    {
        drive.setPoseEstimate(newPos);
    }
    public void forward(double dist)
    {
        forward(drive.getPoseEstimate(),dist);
    }
    public void backward(Pose2d iniPos, double dist)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(iniPos)
                        .back(dist)
                        .build()
        );
    }
    public void backward(double dist)
    {
        backward(drive.getPoseEstimate(),dist);
    }
    public void strafeL(Pose2d iniPos, double dist)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(iniPos)
                        .strafeLeft(dist)
                        .build()
        );
    }
    public void strafeL(double dist)
    {
        strafeL(drive.getPoseEstimate(),dist);
    }
    public void strafeR(Pose2d iniPos, double dist)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(iniPos)
                        .strafeRight(dist)
                        .build()
        );
    }
    public void strafeR(double dist)
    {
        strafeR(drive.getPoseEstimate(),dist);
    }
    public void turnR(double deg)
    {
        drive.turn(-Math.toRadians(deg));
    }
    public void turnL(double deg)
    {
        drive.turn(Math.toRadians(deg));
    }
    public void followLineToSpline(Pose2d iniPos, Pose2d finalPos)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(iniPos)
                    .lineToSplineHeading(finalPos)
                    .build()
        );
    }
    public void followLineToSpline(Pose2d finalPos)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(finalPos)
                        .build()
        );
    }
    public void followSplineToSpline(Pose2d iniPos, Pose2d finalPos,double finTang)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(iniPos)
                        .splineToSplineHeading(finalPos,finTang)
                        .build()
        );
    }
    public void followLineToConstant(Pose2d iniPos, Pose2d finalPos)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(iniPos)
                        .lineToConstantHeading(convertPose2Vector(finalPos))
                        .build()
        );
    }
    public void followLineToConstant(Pose2d iniPos, Vector2d finalVec)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(iniPos)
                        .lineToConstantHeading(finalVec)
                        .build()
        );
    }



    public void shutDown()
    {
        // TODO implement ShutDown on RR Ctrl
    }
    public void doForwardHaltableTrajectory(double distMax, double brakingDist, double brakeVel, double brakeDecel, Boolean signal, Object mutex)
    {
        boolean isRunning = true;
        startInterruptibleTrajVar();
        boolean isForwarding = false;
        boolean isBraking = false;
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(brakeVel, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(brakeDecel);


        new Thread(()->{Boolean tmpIsRunning = Boolean.getBoolean("true");
            while(getIfInterruptibleTraj()) {
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
        synchronized (internalMutex)
        {isRunningInterruptibleTraj = true;}
    }
    private void haltInterruptibleTrajVar()
    {
        synchronized (internalMutex){
        isRunningInterruptibleTraj = false;}
    }
    private boolean getIfInterruptibleTraj()
    {
        synchronized (internalMutex) {
        return isRunningInterruptibleTraj;}
    }


    public static Vector2d convertPose2Vector(Pose2d pose)
    {
        return new Vector2d(pose.getX(),pose.getY());
    }
    public static Pose2d convertVector2Pose(Vector2d vec)
    {
        return convertVector2Pose(vec,0);
    }
    public static Pose2d convertVector2Pose(Vector2d vec,double heading)
    {
        return new Pose2d(vec.getX(),vec.getY(),heading);
    }

}
