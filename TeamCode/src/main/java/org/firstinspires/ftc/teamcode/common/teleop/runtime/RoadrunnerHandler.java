package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.blueIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.drive;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.movementHandler;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideHandler;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousBrain;
import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideTarget;

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
    private final TrajectoryVelocityConstraint turnVelocityConstraint = SampleMecanumDrive.getVelocityConstraint(30, 2, DriveConstants.TRACK_WIDTH);
    private final TrajectoryAccelerationConstraint turnAccelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

    private final TrajectoryVelocityConstraint quickMoveVelocityConstraint = SampleMecanumDrive.getVelocityConstraint(80, 3, DriveConstants.TRACK_WIDTH);
    private final TrajectoryAccelerationConstraint quickMoveAccelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

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
                        .lineToSplineHeading(target, turnVelocityConstraint, turnAccelerationConstraint)
                        .build());
                Log.d(interfaceTag, "Finished pivoting");

            case WAREHOUSE_AUTO_TURN:
                Log.d(interfaceTag, "Going Forward");


                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(14,65.5,0), quickMoveVelocityConstraint, quickMoveAccelerationConstraint)
                                .splineToSplineHeading(new Pose2d(5.58,64.47,-Math.toRadians(55)), Math.PI)
                                .build());


                Log.d(interfaceTag, "Extending Slide");
                // Extend Drop Retract
                slideController.extendSlide();

//                slideController.extendDropRetract(SlideTarget.TOP_DEPOSIT, OfficialTeleop.gamepad);
//                Log.d(interfaceTag, "Turning back onto the wall");
//                // Turn Back Onto The Wall
//                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .lineToSplineHeading(new Pose2d(14,80,0), quickTurnVelocityConstraint, quickTurnAccelerationConstraint)
//                        .build());
//
//                Log.d(interfaceTag, "Going back into the warehouse");
//                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .addTemporalMarker(1, () -> {
//                            Log.d(interfaceTag, "Dropping blue intake");
//                            // Drop Blue Intake
//                            blueIntake.setState(IntakeState.BASE);
//                        })
//                        .forward(24)
//                        .build());

                break;

            case CURRENT_POSITION_TO_DEPOSIT:
                Log.d(interfaceTag, "Im about to go from here to there (ong)");
                Log.d(interfaceTag, "michael says hi uwu :3");

                // drive.followTrajectory(
                //         drive.trajectoryBuilder(drive.getPoseEstimate())
                //                 .splineToSplineHeading(new Pose2d())
                // );

                break;

        }
        scheduledMovement = MovementTypes.EMPTY;
        movementHandler.movementEnabled = true;
        movementHandler.autonomousRunning = false;
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
        if (movementHandler.autonomousRunning || isBusy) {
            return;
        }
        isBusy = true;
        movementHandler.movementEnabled = false;
        slideHandler.slideMovementEnabled = false;
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
