package org.sbs.bears.robotframework.controllers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

@Config
public class JRR {

    /**
     * This boolean is an internal indicator whether the interuptible trajectory is in progress.
     */
    boolean isRunningInterruptibleTraj = false;

    /**
     * This is the RR Drive object for use with the API.
     */
    protected JMD drive;

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
     *
     * @param hardwareMap the FTC HardwareMap given through the OpMode which instantiates Robot, which instantiates this.
     */
    public JRR(HardwareMap hardwareMap, Gamepad gamepad) {
        // initialize everyone.
        this.drive = new JMD(hardwareMap, gamepad);
        this.dashboard = FtcDashboard.getInstance();
        this.runner = drive.trajectorySequenceRunner;
    }

    public JMD getDrive() {
        return drive;
    }

    /**
     * This method returns the robot's current position using RoadRunner.
     *
     * @return the current position of the robot in Pose2d format (RR).
     */
    public Pose2d getCurrentPosition() {
        return drive.getPoseEstimate();
    }

    /**
     * This method sets the robot's position to the specified value.
     *
     * @param newPos the new position of the robot in Pose2d format (RR).
     */
    public void setCurrentPosition(Pose2d newPos) {
        // just call API.
        drive.setPoseEstimate(newPos);
    }

    public void stopTrajectory() {
        runner.cancelTraj();
    }

    public void stopRobot() {
        runner.cancelTraj();
        drive.isRunningFollowTrajectory = false;
        drive.setWeightedDrivePower(new Pose2d()); // set zero power forced.
    }

    public static Vector2d convertPose2Vector(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }

    public static Pose2d convertVector2Pose(Vector2d vec) {
        return convertVector2Pose(vec, 0);
    }

    public static Pose2d convertVector2Pose(Vector2d vec, double heading) {
        return new Pose2d(vec.getX(), vec.getY(), heading);
    }
}
