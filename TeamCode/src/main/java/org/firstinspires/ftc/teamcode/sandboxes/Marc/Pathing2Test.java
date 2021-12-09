package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
@TeleOp(name="U - PathingTuner")
public class Pathing2Test extends LinearOpMode {
    // MARK - Class Variables

    public static int DISTANCE_STRAIGHT = 48;
    public static int DISTANCE_STRAFE = 48;
    public static int TURN_AMOUNT = 90;
    public static int SPLINE_dX = 48;
    public static int SPLINE_dY = 48;
    public static int SPLINE_dH = 90;
    public static int SPLINE_FINTAN = 0;


    /**
     * This is the object which allows us to use RR pathing utilities.
     */
    SampleMecanumDrive drive;
    /**
     * This is the object representing the state. It is <code>volatile</code> in order to ensure
     * multithreading works as expected.
     */
    volatile AutonomousStates4 state = AutonomousStates4.GAMEPAD; // gamepad during init


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
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Thread to report data through telemetry independently of state
        new Thread(()->{
            while(opModeIsActive()) {
                Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.addData("version", 1);
            synchronized (stateMutex) {telemetry.addData("State",state);}
            telemetry.update();

            }

        }).start();
        // start the machine
        //synchronized (stateMutex) {state = AutonomousStates3.One_FORWARD1;}
        drive.setPoseEstimate(new Pose2d()); // set start position!
        while(opModeIsActive() && !isStopRequested()) {
            // MARK - Loop Code
            if(gamepad1.a) {
                synchronized (stateMutex) {state = AutonomousStates4.STRAIGHT_TEST;}
            }
            if(gamepad1.b) {
                synchronized (stateMutex) {state = AutonomousStates4.SPLINE_TEST;}
            }
            if(gamepad1.x) {
                synchronized (stateMutex) {state = AutonomousStates4.TURN_TEST;}
            }
            if(gamepad1.y) {
                synchronized (stateMutex) { state = AutonomousStates4.STRAFE_TEST;}
            }
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
            case GAMEPAD:
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
                drive.update();
            case STRAIGHT_TEST:
                // prepare trajectory: 72 inch forward
                drive.setPoseEstimate(new Pose2d());
                Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(), false)
                        .forward(DISTANCE_STRAIGHT)
                        .build();
                drive.followTrajectory(traj1);
                drive.update();
                // change state
                synchronized (stateMutex) {
                    state = AutonomousStates4.GAMEPAD;
                }
            case TURN_TEST:
                // NOTE turning is not a trajectory
                drive.setPoseEstimate(new Pose2d());
                drive.turn(Math.toRadians(TURN_AMOUNT));
                drive.update();
                synchronized (stateMutex) {
                    state = AutonomousStates4.GAMEPAD;
                }
            case STRAFE_TEST:
                drive.setPoseEstimate(new Pose2d());
                Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(), false)
                        .strafeLeft(DISTANCE_STRAFE)
                        .build();
                drive.followTrajectory(traj4);
                drive.update();
                synchronized (stateMutex) {
                    state = AutonomousStates4.GAMEPAD;
                }
            case SPLINE_TEST:
                drive.setPoseEstimate(new Pose2d());
                Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(), false)
                        .splineToSplineHeading(new Pose2d(SPLINE_dX,SPLINE_dY,SPLINE_dH),SPLINE_FINTAN)
                        .build();
                drive.followTrajectory(traj5);
                drive.update();
                synchronized (stateMutex) {
                    state = AutonomousStates4.GAMEPAD;
                }
        }
    }


}

/**
 * This enum contains the possible states the State Machine can have.
 * @author Marc N
 * @version 4.0
 */
enum AutonomousStates4 {
   STOPPED,
   GAMEPAD,
   STRAIGHT_TEST,
    TURN_TEST,
    STRAFE_TEST,
    SPLINE_TEST
}