package org.sbs.bears.robotframework.controllers;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;


import com.coyote.framework.core.geometry.Pose2d;
import com.coyote.framework.core.geometry.Vector2d;
import com.coyote.framework.core.path.LineSegment;
import com.coyote.framework.core.path.ParametricCurve;
import com.coyote.framework.core.path.Path;
import com.coyote.framework.core.path.PathSegment;
import com.coyote.framework.core.profile.MotionProfile;
import com.coyote.framework.core.profile.MotionSegment;
import com.coyote.framework.core.profile.MotionState;
import com.coyote.framework.core.trajectory.Trajectory;
import com.coyote.framework.core.trajectory.TrajectoryBuilder;
import com.coyote.framework.core.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.coyote.framework.core.trajectory.constraints.TrajectoryVelocityConstraint;
import com.coyote.framework.core.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousBrain;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.List;

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
    public static double VEL_ACCOUNT_CT = 8/35.0;

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



    public SampleMecanumDrive getDrive()
    {
        return drive;
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

    public void forwardAsync(double dist,double vel) {
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(vel, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

        drive.followTrajectoryAsync(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(dist,velocityConstraint,accelerationConstraint)
                        .build()
        );

    }
    public void forward(double dist,double vel) {
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(vel, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(dist,velocityConstraint,accelerationConstraint)
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
    public void followLineToSpline(Pose2d finalPos,double vel)
    {
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(vel, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(finalPos,velocityConstraint,accelerationConstraint)
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
    public void followLineToSplineAsync(Pose2d finalPos)
    {
        drive.followTrajectoryAsync(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(finalPos)
                        .build()
        );
        new Thread(()->{drive.waitForIdle();}).start();
    }
    public void stopTrajectory()
    {
        runner.cancelTraj();
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

    public void stopRobot()
    {
        runner.cancelTraj();
        drive.setWeightedDrivePower(new Pose2d()); // set zero power forced.
    }



    public void doForwardHaltableTrajectory(double distMax, double brakingDist, double brakeVel, double brakeDecel, Boolean signal, Object mutex)
    {
        Log.d("HaltableTrajectoryRunner","init");
        boolean isRunning = true;
        startInterruptibleTrajVar();
        boolean isForwarding = false;
        boolean isBraking = false;
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(brakeVel, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH);
        TrajectoryVelocityConstraint velocityConstraint2 = SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(brakeDecel);
        TrajectoryAccelerationConstraint accelerationConstraint2 = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

        Log.d("HaltableTrajectoryRunner","start halting thread");

        new Thread(()->{Boolean tmpIsRunning = Boolean.getBoolean("true");
            while(getIfInterruptibleTraj()) {
                synchronized (mutex) {
                    if(signal) {
                        runner.cancelTraj();
                        Log.d("HaltableTrajectoryRunner","traj halted by signal");
                        return;
                    }
                }
            }
        }).start();

        Log.d("HaltableTrajectoryRunner","prepare traj");

        Pose2d iniPos = drive.getPoseEstimate();
        double iniX = iniPos.getX();
        Log.d("HaltableTrajectoryRunner","start traj");
        Log.d("HaltableTrajectoryRunner","iniPos: " + iniPos.toString());

        double iniTime = NanoClock.system().seconds();
        Trajectory trajForward = drive.trajectoryBuilder(iniPos)
                //.lineToSplineHeading(new Pose2d(distMax+iniX,iniPos.getY(),iniPos.getHeading()))
                .forward(distMax,velocityConstraint2,accelerationConstraint2)
                .build();
        isForwarding = true;
        drive.followTrajectory(trajForward); // interruptible
        isForwarding = false;
        haltInterruptibleTrajVar();
        Log.d("HaltableTrajectoryRunner","traj halted");
        isBraking = true;
        Log.d("HaltableTrajectoryRunner","braking traj start");
        /*double currentX = drive.getPoseEstimate().getX();
        Trajectory trajBrake = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(brakingDist+currentX-iniX,velocityConstraint,accelerationConstraint)
                .build();
        drive.followTrajectoryTime(trajBrake,iniTime);*/
        Log.d("HaltableTrajectoryRunner","braking traj end");
        Log.d("HaltableTrajectoryRunner","done");
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

    public void followLineToSplineWithIniVel(Pose2d target){
        Pose2d vels = drive.getPoseVelocity();
        double v0 = Math.sqrt(vels.getX()*vels.getX() + vels.getY()*vels.getY()); // ini vel mag
        followLineToSplineWithIniVel(target,v0);
    }
    public void followLineToSplineWithIniVel(Pose2d target, double iniVel){
        Pose2d current = getPos();
        Pose2d vels = drive.getPoseVelocity();
        // Create a spoofed trajectory
        double delta = iniVel*VEL_ACCOUNT_CT; // to determine
        Trajectory spoofed = createSpoofedTraj(current,vels,new Vector2d(delta,0),0.5);
        // Follow the trajectory sequence
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);
        TrajectoryBuilder engine = new TrajectoryBuilder(new Pose2d(current.getX()+delta,current.getY(),current.getHeading()),0.0,spoofed,0.0,velocityConstraint,accelerationConstraint,new MotionState(0,iniVel,0),0.25);

        drive.followTrajectory(engine
                        .lineToSplineHeading(target) // this should now be smooth!!!)
                        .build());
    }
    public void doOptimizedTeleOpDepositTraj(TrajectoryVelocityConstraint quickMoveVelocityConstraint, TrajectoryAccelerationConstraint quickMoveAccelerationConstraint)
    {
        Pose2d vels = drive.getPoseVelocity();
        double v0 = Math.sqrt(vels.getX()*vels.getX() + vels.getY()*vels.getY()); // ini vel mag
        Pose2d current = getPos();
        // Create a spoofed trajectory
        double delta = v0*VEL_ACCOUNT_CT; // to determine
        Trajectory spoofed = createSpoofedTraj(current,vels,new Vector2d(delta,0),0.5);
        // Follow the trajectory sequence
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);
        TrajectoryBuilder engine = new TrajectoryBuilder(new Pose2d(current.getX()+delta,current.getY(),current.getHeading()),0.0,spoofed,0.0,velocityConstraint,accelerationConstraint,new MotionState(0,v0,0),0.25);

        drive.followTrajectory(engine
                .lineToSplineHeading(new Pose2d(14,65.5,0), quickMoveVelocityConstraint, quickMoveAccelerationConstraint)
                .splineToSplineHeading(new Pose2d(5.58,64.47,-Math.toRadians(55)), Math.PI)
                .build());
    }

    /*


    */
    public Trajectory createSpoofedTraj(Pose2d inipos, Pose2d inivel, Vector2d dPos, double dt) {
        double x = inipos.getX(); // current pos
        double y = inipos.getY(); // current pos
        double h = inipos.getHeading(); // current pos
        double dx = dPos.getX(); // delta
        double dy = dPos.getY(); // delta
        double v0 = Math.sqrt(inivel.getX()*inivel.getX() + inivel.getY()*inivel.getY()); // ini vel mag


        // Create a motion profile segment "spoofed"
        MotionState state = new MotionState(x,v0,0,0);
        MotionSegment motionSegment = new MotionSegment(state,dt);
        // Make a new segment list and add the spoofed segment
        List<MotionSegment> motionSegs = new ArrayList<>(1);
        motionSegs.add(motionSegment);
        // Create a motion profile based off of the motion segments
        MotionProfile motionProfile = new MotionProfile(motionSegs);

        // Create the curve to follow
        ParametricCurve curve = new LineSegment(new Vector2d(x,y),new Vector2d(x+dx,y+dy));
        // Make a path segment based off of the curve and add it to a path segment list
        PathSegment pathSegment = new PathSegment(curve);
        List<PathSegment> pathSegs = new ArrayList<>(1);
        pathSegs.add(pathSegment);

        // Create a path based off of the path segment list
        Path path = new Path(pathSegs);

        // Build the spoofed trajectory
        Trajectory spoofed = new Trajectory(path,motionProfile);
        return spoofed;
    }
    public void doBlueDepositTrajectory()
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(AutonomousBrain.startPositionBlue)
                        .splineToSplineHeading(AutonomousBrain.depositPositionAllianceBlue2,Math.PI)
                        .build()
        );
    }
    public void doBlueAutonomousParkingTrajectory()
    {
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToSplineHeading(AutonomousBrain.resetPositionB4WarehouseBlue,Math.PI)
                        .lineToSplineHeading(AutonomousBrain.parkingPositionBlue)
                        .build()
        );
    }

    public void haltTrajectory()
    {
        runner.cancelTraj();
    }

}
