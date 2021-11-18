package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This OpMode performs the following path using a state machine:
 * 6 inches forward;
 * turn right 90;
 * 8 inches strafe right;
 * 24 inches forward;
 * 24 inches backward;
 * 24 inches forward;
 * 24 inches backward;
 * 24 inches forward;
 * 24 inches backward;
 * 24 inches forward;
 * 24 inches backward;
 * 24 inches forward;
 * 24 inches backward;
 * @author Marc N
 * @version 5.1
 */
@Autonomous(name="A - Sample Autonomous Path")
public class SampleAutonPath extends LinearOpMode {
    // MARK - Class Variables

    /**
     * This is the object which allows us to use RR pathing utilities.
     */
    SampleMecanumDrive drive;
    /**
     * This is the object representing the state. It is <code>volatile</code> in order to ensure
     * multithreading works as expected.
     */
    volatile AutonomousStates state = AutonomousStates.STOPPED;
    /**
     * This is the mutex object ensuring that concurrent threads don't override each other.
     */
    Object stateMutex = new Object();

    // tmp position
    Pose2d loc2;

    /**
     * This method consists of the OpMode initialization, start, and loop code.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // MARK - Initialization
        drive= new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(2000);
        waitForStart();

        // Thread to report data through telemetry independently of state
        new Thread(()->{
            while(opModeIsActive()) {
                Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            synchronized (stateMutex) {telemetry.addData("State",state);}
            telemetry.update();

            }

        }).start();
        // start the machine
        synchronized (stateMutex) {state = AutonomousStates.ONE_BACKWARD;}
        drive.setPoseEstimate(new Pose2d()); // set start position!
        while(opModeIsActive() && !isStopRequested()) {
            // MARK - Loop Code
            doRobotStateAction();
            drive.update();
        }

    }
    // MARK  - Helper Methods
    /**
     * This method contains the state actions for each state.
     * It is called each time in the loop.
     * Note that the state changes automatically at the end of the state actions.
     */
    public void doRobotStateAction()
    {
        switch(state) {
            case STOPPED:
                return;
            case ONE_BACKWARD:
                Pose2d loc = drive.getPoseEstimate();
                Trajectory traj1 = drive.trajectoryBuilder(loc)
                        .back(40)
                        .build();
                drive.followTrajectory(traj1);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.TWO_FORWARD;}
            case TWO_FORWARD:
                // NOTE turning is not a trajectory
                Pose2d loc2 = drive.getPoseEstimate();
                Trajectory traj2 = drive.trajectoryBuilder(loc2)
                        .forward(40)
                        .build();
                drive.followTrajectory(traj2);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.THREE_TURN35;}
            case THREE_TURN35:
                Pose2d loc3 = drive.getPoseEstimate();
                Trajectory traj3 = drive.trajectoryBuilder(loc3)
                        .lineToSplineHeading(new Pose2d(loc3.getX() + (9-9*Math.sqrt(2)*Math.cos(Math.toRadians(80))),loc3.getY() + (9-9*Math.sqrt(2)*Math.sin(Math.toRadians(80))),loc3.getHeading()+Math.toRadians(35)))
                        .build();
                drive.followTrajectory(traj3);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.FOUR_WAIT;}
            case FOUR_WAIT:
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                synchronized (stateMutex) {state = AutonomousStates.FIVE_TURNBACK;}
            case FIVE_TURNBACK:
                Pose2d loc5 = drive.getPoseEstimate();
                Trajectory traj5 = drive.trajectoryBuilder(loc5)
                        .lineToSplineHeading(new Pose2d(loc5.getX() - (9-9*Math.sqrt(2)*Math.cos(Math.toRadians(80))),loc5.getY() - (9-9*Math.sqrt(2)*Math.sin(Math.toRadians(80))),loc5.getHeading()-Math.toRadians(35)))
                        .build();
                drive.followTrajectory(traj5);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.STOPPED;}

        }
    }


}

/**
 * This enum contains the possible states the State Machine can have.
 * @author Marc N
 * @version 1.0
 */
 enum AutonomousStates {
    STOPPED,
    GAMEPAD,
    ONE_BACKWARD,
    TWO_FORWARD,
    THREE_TURN35,
    FOUR_WAIT,
    FIVE_TURNBACK,


}