package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.StandardDrive;
import org.sbs.bears.controller.ColorStripController;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@TeleOp(name="A - Autonomous Simulator (X)")
public class AutonSimulator extends LinearOpMode {
    StandardDrive drive;
    ColorStripController colorCtrl;

    // Published parameters
    public static boolean instaStop = false;
    public static double stoppingDistance = 5; // in
    public static double brakeDecel = DriveConstants.MAX_ACCEL;
    public static double brakeVel = DriveConstants.MAX_VEL;

    // constraints
    TrajectoryVelocityConstraint velModif = StandardDrive.getVelocityConstraint(brakeVel, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
    TrajectoryAccelerationConstraint accelModif =  StandardDrive.getAccelerationConstraint(brakeDecel);

    Object stateMutex = new Object();
    enum State {
        FORWARD,
        BRAKE,
        BACKWARDS,
        STOPPED
    }
    State state = State.STOPPED;
    @Override
    public void runOpMode() throws InterruptedException {
        NanoClock clock = NanoClock.system();
        drive = new StandardDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        colorCtrl = new ColorStripController(hardwareMap);
        colorCtrl.setDarkBlue();
        waitForStart();
        new Thread(()->{
            boolean qX = false;
            while(opModeIsActive() && !isStopRequested()) {
                if(gamepad1.x && !qX) {
                    synchronized (stateMutex) {
                        if(state.equals(State.FORWARD)) {
                            colorCtrl.setRed();
                            drive.trajectorySequenceRunner.cancelTraj();
                        }
                    }
                    qX = true;
                }
                else if(!gamepad1.x && qX) {
                    qX = false;
                }
            }
        }).start();
        while(opModeIsActive() && !isStopRequested()) {
            synchronized (stateMutex) {
            state = State.FORWARD;
            colorCtrl.setForest();}
            double time = clock.seconds();
            double iniX = drive.getPoseEstimate().getX();
            goForward(); // could be interrupted.
            if(!instaStop) {
            synchronized (stateMutex) {
            state = State.BRAKE;}
            double currentX = drive.getPoseEstimate().getX();
            double deltaX = currentX - iniX;
            colorCtrl.setParty();
            goSlower(time, deltaX);
            }
            synchronized (stateMutex) {
            state = State.BACKWARDS;}
            colorCtrl.setOcean();
            goHome();
        }
    }
    public void goForward() {
        Trajectory trajForward = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(72,0,0))
                .build();
        drive.followTrajectory(trajForward);
    }
    public void goSlower(double time, double deltaX) {
        Trajectory trajForward = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(stoppingDistance+deltaX,velModif,accelModif)
                .build();
        drive.followTrajectoryTime(trajForward, time);
    }
    public void goHome() {
        Trajectory backHome = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d())
                .build();
        drive.followTrajectory(backHome);
    }
}
