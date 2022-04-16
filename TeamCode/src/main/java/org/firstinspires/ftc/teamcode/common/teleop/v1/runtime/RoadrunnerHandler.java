package org.firstinspires.ftc.teamcode.common.teleop.v1.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.v1.BlueTeleOp.drive;
import static org.firstinspires.ftc.teamcode.common.teleop.v1.BlueTeleOp.slideController;
import static org.firstinspires.ftc.teamcode.common.teleop.v1.BlueTeleOp.slideHandler;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.common.teleop.v1.Configuration;
import org.firstinspires.ftc.teamcode.drive.DriveConstantsMain;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * Brings Automatic Movement To TeleOp
 */
public class RoadrunnerHandler {

    public boolean isBusy = false;

    /**
     * Movement Types to Schedule
     */
    enum MovementTypes {
        EMPTY(0),
        LEFT(Configuration.inchesLeft),
        RIGHT(Configuration.inchesRight),
        FORWARD(Configuration.inchesForward),
        BACK(Configuration.inchesBack),
        TURN_ABOUT_WHEEL(0),
        WAREHOUSE_AUTO_TURN(0),
        CURRENT_POSITION_TO_DEPOSIT(0);

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
    private final TrajectoryVelocityConstraint turnVelocityConstraint = SampleMecanumDrive.getVelocityConstraint(30, 2, DriveConstantsMain.TRACK_WIDTH);
    private final TrajectoryAccelerationConstraint turnAccelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstantsMain.MAX_ACCEL);

    /**
     * Internal executor
     */
    private Thread movementExecutor = new Thread(() -> {
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
                        .lineToSplineHeading(target, turnVelocityConstraint, turnAccelerationConstraint)
                        .build());
                Log.d(interfaceTag, "Finished pivoting");

            case WAREHOUSE_AUTO_TURN:
                Log.d(interfaceTag, "Going Forward");
                // drive.setPoseEstimate(new Pose2d(28.5, 65.5, 0));
                drive.setPoseEstimate(new Pose2d(14, 65.5, 0));

                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                // .lineToSplineHeading(new Pose2d(14,65.5,0), quickMoveVelocityConstraint, quickMoveAccelerationConstraint)
                                .splineToSplineHeading(new Pose2d(5.58,64.47,-Math.toRadians(55)), Math.PI)
                          //      .splineToSplineHeading(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(),-Math.toRadians(55)), Math.PI)

                                .build());


                Log.d(interfaceTag, "Extending Slide");
                // Extend Drop Retract
                slideController.extendSlide();
                break;

            case CURRENT_POSITION_TO_DEPOSIT:

                break;

        }
        scheduledMovement = MovementTypes.EMPTY;
        slideHandler.slideMovementEnabled = true;
        isBusy = false;
        requestKill();
        Log.d(interfaceTag, "Interface still running");
    });

    /**
     * Schedule A Movement
     */
    // TODO: Add an indicator showing if the robot took the movement
    public void scheduleMovement(MovementTypes movementType) {
        isBusy = true;
        slideHandler.slideMovementEnabled = false;
        if (scheduledMovement != MovementTypes.EMPTY) {
            scheduledMovement = MovementTypes.EMPTY;
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
        slideHandler.slideMovementEnabled = true;
    }
}
