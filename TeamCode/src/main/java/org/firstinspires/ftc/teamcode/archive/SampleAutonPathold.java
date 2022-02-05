package org.firstinspires.ftc.teamcode.archive;

import com.coyote.framework.core.geometry.Pose2d;
import com.coyote.framework.core.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class SampleAutonPathold extends LinearOpMode {
    // MARK - Class Variables

    /**
     * This is the object which allows us to use RR pathing utilities.
     */
    SampleMecanumDrive drive;
    /**
     * This is the object representing the state. It is <code>volatile</code> in order to ensure
     * multithreading works as expected.
     */
    volatile AutonomousStatesold state = AutonomousStatesold.STOPPED;
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
        synchronized (stateMutex) {state = AutonomousStatesold.One_STAGE_ADVANCE;}
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
            case One_STAGE_ADVANCE:
                // prepare trajectory: 6 inch forward
                Pose2d loc = drive.getPoseEstimate();
                Trajectory traj1 = drive.trajectoryBuilder(loc)
                        .lineToSplineHeading(new Pose2d(loc.getX() + 6,loc.getY(),loc.getHeading()))
                        .build();
                drive.followTrajectory(traj1);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStatesold.Two_ROTATE;}
            case Two_ROTATE:
                // NOTE turning is not a trajectory
                loc2 = drive.getPoseEstimate();
                drive.turn(Math.toRadians(90));
                drive.update();
                synchronized (stateMutex) {state = AutonomousStatesold.Three_BACK_TO_WALL;}
            /*case Twobis_FIXPOS: // skipped
                Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(loc2.getX(),loc2.getY(),loc2.getHeading() + Math.toRadians(90)))
                        .build();
                drive.followTrajectory(traj2);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.Three_BACK_TO_WALL;}*/
            case Three_BACK_TO_WALL:
                Pose2d loc3 = drive.getPoseEstimate();
                Trajectory traj3 = drive.trajectoryBuilder(loc3)
                        .lineToSplineHeading(new Pose2d(loc3.getX() - 8,loc3.getY(),loc2.getHeading()+Math.toRadians(90)))
                        .build();
                drive.followTrajectory(traj3);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStatesold.FORWARD1;}
            case FORWARD1:
                Pose2d loc4 = drive.getPoseEstimate();
                Trajectory traj4 = drive.trajectoryBuilder(loc4)
                        .lineToSplineHeading(new Pose2d(loc4.getX(),loc4.getY()+24,loc4.getHeading()))
                        .build();
                drive.followTrajectory(traj4);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStatesold.BACKWARD1;}
            case BACKWARD1:
                Pose2d loc5 = drive.getPoseEstimate();
                Trajectory traj5 = drive.trajectoryBuilder(loc5)
                        .lineToSplineHeading(new Pose2d(loc5.getX(),loc5.getY()-24,loc5.getHeading()))
                        .build();
                drive.followTrajectory(traj5);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStatesold.FORWARD3;}
            case FORWARD3:
                Pose2d loc6 = drive.getPoseEstimate();
                Trajectory traj6 = drive.trajectoryBuilder(loc6)
                        .lineToSplineHeading(new Pose2d(loc6.getX(),loc6.getY()+24,loc6.getHeading()))
                        .build();
                drive.followTrajectory(traj6);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStatesold.BACKWARD3;}
            case BACKWARD3:
                Pose2d loc7 = drive.getPoseEstimate();
                Trajectory traj7 = drive.trajectoryBuilder(loc7)
                        .lineToSplineHeading(new Pose2d(loc7.getX(),loc7.getY()-24,loc7.getHeading()))
                        .build();
                drive.followTrajectory(traj7);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStatesold.FORWARD4;}
            case FORWARD4:
                Pose2d loc8 = drive.getPoseEstimate();
                Trajectory traj8 = drive.trajectoryBuilder(loc8)
                        .lineToSplineHeading(new Pose2d(loc8.getX(),loc8.getY()+24,loc8.getHeading()))
                        .build();
                drive.followTrajectory(traj8);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStatesold.BACKWARD4;}
            case BACKWARD4:
                Pose2d loc9 = drive.getPoseEstimate();
                Trajectory traj9 = drive.trajectoryBuilder(loc9)
                        .lineToSplineHeading(new Pose2d(loc9.getX(),loc9.getY()-24,loc9.getHeading()))
                        .build();
                drive.followTrajectory(traj9);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStatesold.FORWARD5;}
            case FORWARD5:
                Pose2d loc10 = drive.getPoseEstimate();
                Trajectory traj10 = drive.trajectoryBuilder(loc10)
                        .lineToSplineHeading(new Pose2d(loc10.getX(),loc10.getY()+24,loc10.getHeading()))
                        .build();
                drive.followTrajectory(traj10);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStatesold.BACKWARD5;}
            case BACKWARD5:
                Pose2d loc11 = drive.getPoseEstimate();
                Trajectory traj11 = drive.trajectoryBuilder(loc11)
                        .lineToSplineHeading(new Pose2d(loc11.getX(),loc11.getY()-24,loc11.getHeading()))
                        .build();
                drive.followTrajectory(traj11);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStatesold.STOPPED;}
        }
    }


}

/**
 * This enum contains the possible states the State Machine can have.
 * @author Marc N
 * @version 2.0
 */
 enum AutonomousStatesold {
    STOPPED,
    GAMEPAD,
    One_STAGE_ADVANCE,
    Two_ROTATE,
     //Twobis_FIXPOS(2.5),
    Three_BACK_TO_WALL,
    FORWARD1,
    BACKWARD1,
    FORWARD2,
    BACKWARD2,
    FORWARD3,
    BACKWARD3,
    FORWARD4,
    BACKWARD4,
    FORWARD5,
    BACKWARD5;

}