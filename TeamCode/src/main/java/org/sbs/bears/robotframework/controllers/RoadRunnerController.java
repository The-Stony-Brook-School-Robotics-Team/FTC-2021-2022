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

/**
 * This class is a wrapper controller for the RoadRunner Library.
 * Please use this controller as a part of the Robot class and do not instantiate it separately.
 * @author Marc D Nichitiu
 * @version 1.0
 */
public class RoadRunnerController {

    /**
     * This boolean is an internal indicator whether the interuptible trajectory is in progress.
     */
    boolean isRunningInterruptibleTraj = false;

    /**
     * This is the RR Drive object for use with the API.
     */
    protected SampleMecanumDrive drive;
    /**
     * This is the RR Dashboard object for use with the API.
     */
    protected FtcDashboard dashboard;
    /**
     * This is the RR Trajectory Runner object for use with the API.
     */
    protected TrajectorySequenceRunner runner;

    /**
     * This is the controller's internal mutex. Do not use with external synchronizations.
     * External synchronizations require an external mutex provided as a parameter, such as in doForwardHaltableTrajectory().
     */
    Object internalMutex = new Object();

    /**
     * This is the constructor for the RR Ctrller.
     * @param hardwareMap the FTC HardwareMap given through the OpMode which instantiates Robot, which instantiates this.
     * @param telemetry the FTC Telemetry object given in the same manner as the hardwareMap.
     */
    public RoadRunnerController(HardwareMap hardwareMap, Telemetry telemetry)
    {
        // initialize everyone.
        this.drive = new SampleMecanumDrive(hardwareMap);
        this.dashboard = FtcDashboard.getInstance();
        this.runner = drive.trajectorySequenceRunner;
    }

    /**
     * This method returns the robot's current position using RoadRunner.
     * @return the current position of the robot in Pose2d format (RR).
     */
    public Pose2d getPos()
    {
        // just call API.
        return drive.getPoseEstimate();
    }
    /**
     * This method sets the robot's position to the specified value.
     * @param newPos the new position of the robot in Pose2d format (RR).
     */
    public void setPos(Pose2d newPos)
    {
        // just call API.
        drive.setPoseEstimate(newPos);
    }

    /**
     * This method runs a simple forward trajectory from the given postion to a certain distance forward.
     * @param iniPos the initial position.
     * @param dist the distance to travel.
     */
    public void forward(Pose2d iniPos, double dist)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(iniPos)
                        .forward(dist)
                        .build()
        );
    }
    /**
     * This method runs a simple forward trajectory from the current postion to a certain distance forward.
     * @param dist the distance to travel.
     */
    public void forward(double dist)
    {
        forward(drive.getPoseEstimate(),dist);
    }
    /**
     * This method runs a simple backward trajectory from the given postion to a certain distance backwards.
     * @param iniPos the initial position.
     * @param dist the distance to travel.
     */
    public void backward(Pose2d iniPos, double dist)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(iniPos)
                        .back(dist)
                        .build()
        );
    }
    /**
     * This method runs a simple backward trajectory from the current postion to a certain distance backwards.
     * @param dist the distance to travel.
     */
    public void backward(double dist)
    {
        backward(drive.getPoseEstimate(),dist);
    }
    /**
     * This method runs a simple left strafing trajectory from the given postion to a certain distance to the left.
     * @param iniPos the initial position.
     * @param dist the distance to travel.
     */
    public void strafeL(Pose2d iniPos, double dist)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(iniPos)
                        .strafeLeft(dist)
                        .build()
        );
    }
    /**
     * This method runs a simple left strafing trajectory from the given postion to a certain distance to the left.
     * @param dist the distance to travel.
     */
    public void strafeL(double dist)
    {
        strafeL(drive.getPoseEstimate(),dist);
    }
    /**
     * This method runs a simple right strafing trajectory from the given postion to a certain distance to the right.
     * @param iniPos the initial position.
     * @param dist the distance to travel.
     */
    public void strafeR(Pose2d iniPos, double dist)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(iniPos)
                        .strafeRight(dist)
                        .build()
        );
    }
    /**
     * This method runs a simple right strafing trajectory from the given postion to a certain distance to the right.
     * @param dist the distance to travel.
     */
    public void strafeR(double dist)
    {
        strafeR(drive.getPoseEstimate(),dist);
    }

    /**
     * This method turns right the specificied amount of degrees.
     * @param deg the number of degrees to turn.
     */
    public void turnR(double deg)
    {
        drive.turn(-Math.toRadians(deg));
    }
    /**
     * This method turns left the specificied amount of degrees.
     * @param deg the number of degrees to turn.
     */
    public void turnL(double deg)
    {
        drive.turn(Math.toRadians(deg));
    }

    /**
     * This method follows a LineToSplineHeading trajectory through RoadRunner.
     * @param iniPos the start position
     * @param finalPos the end position
     */
    public void followLineToSpline(Pose2d iniPos, Pose2d finalPos)
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(iniPos)
                    .lineToSplineHeading(finalPos)
                    .build()
        );
    }

    public void followSplineTrajWarehouse(boolean qBlue)
    {
        if (qBlue) {
            TrajectorySequence seq = drive.trajectorySequenceBuilder(new Pose2d(-6,66,0))
                    .forward(45)
                    .lineToSplineHeading(new Pose2d(39,34,-Math.PI/2))
                    .strafeLeft(32)
                    .build();
            drive.followTrajectorySequence(seq);
            // final pos is 71, 34, -Math.PI/2
        }
        else {
            TrajectorySequence seq = drive.trajectorySequenceBuilder(new Pose2d(-6,-66,0))
                    .forward(45)
                    .lineToSplineHeading(new Pose2d(39,-34,Math.PI/2))
                    .strafeRight(32)
                    .build();
            drive.followTrajectorySequence(seq);
            // final pos is 71, -34, Math.PI/2
        }
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
