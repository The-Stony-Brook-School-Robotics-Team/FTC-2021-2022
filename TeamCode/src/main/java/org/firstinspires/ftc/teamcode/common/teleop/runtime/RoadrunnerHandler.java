package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.drive;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.movementHandler;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideHandler;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * Brings Automatic Movement To TeleOp
 */
public class RoadrunnerHandler {

    /**
     * Movement Types to Schedule
     */
    enum MovementTypes {
        EMPTY(0),
        LEFT(Configuration.inchesLeft),
        RIGHT(Configuration.inchesRight),
        FORWARD(Configuration.inchesForward),
        BACK(Configuration.inchesBack),
        TURN_ABOUT_WHEEL(0);

        private int inches;

        private int getInches() {
            return this.inches;
        }

        MovementTypes(int inches) {
            this.inches = inches;
        }

    }

    /**
     * Tags
     */
    private MovementTypes scheduledMovement = null;
    private String interfaceTag = "RoadRunner Handler";

    /**
     * Kill all management threads
     */
    public void sendKillSignal() {
        movementExecutor.interrupt();
    }

    /**
     * Movement Stuff
     */
    private final TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(30, 2, DriveConstants.TRACK_WIDTH);
    private final TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);


    /**
     * Internal executor
     */
    private Thread movementExecutor = new Thread(() -> {
        movementHandler.autonomousRunning = true;
        switch (scheduledMovement) {
            case LEFT:
                Trajectory left = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeLeft(scheduledMovement.getInches())
                        .build();
                drive.followTrajectory(left);
                Log.d(interfaceTag, "Moving " + String.valueOf(scheduledMovement.getInches()) + " inches left");
                break;
            case RIGHT:
                Trajectory right = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeRight(scheduledMovement.getInches())
                        .build();
                drive.followTrajectory(right);
                Log.d(interfaceTag, "Moving " + String.valueOf(scheduledMovement.getInches()) + " right left");
                break;

            case FORWARD:
                Trajectory forward = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(scheduledMovement.getInches())
                        .build();
                drive.followTrajectory(forward);
                Log.d(interfaceTag, "Moving " + String.valueOf(scheduledMovement.getInches()) + " inches forward");
                break;
            case BACK:
                Trajectory back = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .back(scheduledMovement.getInches())
                        .build();
                drive.followTrajectory(back);
                Log.d(interfaceTag, "Moving " + String.valueOf(scheduledMovement.getInches()) + " inches backwards");
                break;

            case TURN_ABOUT_WHEEL:
                Log.d(interfaceTag, "Starting to pivot");
                drive.setPoseEstimate(new Pose2d(14, 65.5, 0));
                Pose2d currentPos = drive.getPoseEstimate();
                Pose2d target = new Pose2d(5.58, 64.47, -Math.toRadians(58));
                drive.followTrajectory(drive.trajectoryBuilder(currentPos)
                        .lineToSplineHeading(target, velocityConstraint, accelerationConstraint)
                        .build());
                Log.d(interfaceTag, "Finished pivoting");
        }
        scheduledMovement = MovementTypes.EMPTY;
        movementHandler.movementEnabled = true;
        movementHandler.autonomousRunning = false;
        slideHandler.slideMovementEnabled = true;
        requestKill();
    });

    /**
     * Schedule A Movement
     */
    // TODO: Add an indicator showing if the robot took the movement
    public void scheduleMovement(MovementTypes movementType) {
        if (movementHandler.autonomousRunning) {
            return;
        }
        movementHandler.movementEnabled = false;
        slideHandler.slideMovementEnabled = false;
        Log.d(interfaceTag, "I FINALLY SET THE MOVEMENT TO FALSE");
        Log.d(interfaceTag, "---- Movement Enabled: " + movementHandler.movementEnabled);
        if (scheduledMovement != MovementTypes.EMPTY) {
            scheduledMovement = MovementTypes.EMPTY;
        }
        if (movementHandler.currentDriverMode != MovementHandler.DriverMode.AUTOMATIC) {
            movementHandler.currentDriverMode = MovementHandler.DriverMode.AUTOMATIC;
        }
        scheduledMovement = movementType;
        movementExecutor.start();
    }

    /**
     * Kills Movement Executor Thread
     */
    public void requestKill() {
        movementExecutor.interrupt();
    }

    /**
     * Soft Kills Movement
     */
    public void softKill() {
        drive.trajectorySequenceRunner.cancelTraj();
        movementExecutor.interrupt();
        movementHandler.autonomousRunning = false;
        movementHandler.currentDriverMode = MovementHandler.DriverMode.DRIVER;
        movementHandler.movementEnabled = true;
        slideHandler.slideMovementEnabled = true;
    }
}
