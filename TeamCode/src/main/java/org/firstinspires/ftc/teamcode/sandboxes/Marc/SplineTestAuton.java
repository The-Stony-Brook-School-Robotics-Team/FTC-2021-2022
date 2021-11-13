package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.BearsUtil.T265Controller;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This OpMode performs the following path using a state machine:
 * Spline to pass between PVC pipes and cubes/spheres pile
 * @author Marc N
 * @version 5.1
 */
@Autonomous(name="A - Spline Test (Marc)")
public class SplineTestAuton extends LinearOpMode {
    // MARK - Class Variables

    /**
     * This is the object which allows us to use RR pathing utilities.
     */
    SampleMecanumDrive drive;
    /**
     * This is the object representing the state. It is <code>volatile</code> in order to ensure
     * multithreading works as expected.
     */
    volatile AutonomousStates2 state = AutonomousStates2.STOPPED;
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
        synchronized (stateMutex) {state = AutonomousStates2.One_SPLINE;}
        drive.setPoseEstimate(new Pose2d()); // set start position!
        while(opModeIsActive() && !isStopRequested()) {
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
            case One_SPLINE:
                drive.setPoseEstimate(new Pose2d(0,65,-Math.PI/2));
                // prepare spline trajectory
                // NOTE: we use splines since then the trajectory is smooth and we minimize time
                Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0, 65, -Math.PI / 2), false)
                        .strafeLeft(24.0)
                        .splineToSplineHeading(new Pose2d(43.0, 48.0, -Math.PI * 3.0 / 4.0), -Math.PI / 4.0)
                        .splineToSplineHeading(new Pose2d(65.0, 24.0, -Math.PI/1.0), -Math.PI / 2.0)
                        .splineToSplineHeading(new Pose2d(65.0, 12.0, -Math.PI), -Math.PI / 2.0)
                        .build();
                drive.followTrajectory(traj1);
                drive.update();
                synchronized (stateMutex) {
                    state = AutonomousStates2.STOPPED;
                }
        }
    }


}

/**
 * This enum contains the possible states the State Machine can have.
 * @author Marc N
 * @version 2.0
 */
enum AutonomousStates2 {
    STOPPED,
    GAMEPAD,
    One_SPLINE,
    Two_SPLINE2

}