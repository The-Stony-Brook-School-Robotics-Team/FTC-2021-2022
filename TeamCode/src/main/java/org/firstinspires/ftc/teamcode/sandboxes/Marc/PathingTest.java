package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This OpMode performs the following path using a state machine:
 * 72 inches forward;
 * turn right 90;
 * 24 inches forward;
 * turn right 90;
 * 72 inches forward.
 * @author Marc N
 * @version 2.2
 */
@Autonomous(name="A - 72R24R72 Pathing Test")
public class PathingTest extends LinearOpMode {
    // MARK - Class Variables

    /**
     * This is the object which allows us to use RR pathing utilities.
     */
    SampleMecanumDrive drive;
    /**
     * This is the object representing the state. It is <code>volatile</code> in order to ensure
     * multithreading works as expected.
     */
    volatile AutonomousStates3 state = AutonomousStates3.STOPPED; // stopped during init


    /**
     * This is the mutex object ensuring that concurrent threads don't override each other.
     */
    Object stateMutex = new Object();

    /**
     * This method consists of the OpMode initialization, start, and loop code.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // MARK - Initialization
        drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(2000);
        waitForStart();
        drive.setPoseEstimate(new Pose2d());

        // Thread to report data through telemetry independently of state
        new Thread(()->{
            while(opModeIsActive()) {
                Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.addData("version", 7);
            synchronized (stateMutex) {telemetry.addData("State",state);}
            telemetry.update();

            }

        }).start();
        // start the machine
        synchronized (stateMutex) {state = AutonomousStates3.One_FORWARD1;}
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
            case One_FORWARD1:
                // prepare trajectory: 72 inch forward
                Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(), false)
                        .lineToSplineHeading(new Pose2d(72,0,0))
                        .build();
                drive.followTrajectory(traj1);
                drive.update();
                // change state
                synchronized (stateMutex) {
                    state = AutonomousStates3.Two_TURN1;
                }
            case Two_TURN1:
                // NOTE turning is not a trajectory
                drive.turn(-Math.PI);
                drive.update();
                synchronized (stateMutex) {
                    state = AutonomousStates3.Three_FORWARD2;
                }
            case Three_FORWARD2:
                Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(72,0,-Math.PI), false)
                        .lineToSplineHeading(new Pose2d(0,0,-Math.PI))
                        .build();
                drive.followTrajectory(traj3);
                drive.update();
                synchronized (stateMutex) {
                    state = AutonomousStates3.STOPPED;
                }

        }
    }


}

/**
 * This enum contains the possible states the State Machine can have.
 * @author Marc N
 * @version 3.0
 */
enum AutonomousStates3 {
   STOPPED,
   GAMEPAD,
   One_FORWARD1,
    Two_TURN1,
    Three_FORWARD2,
}