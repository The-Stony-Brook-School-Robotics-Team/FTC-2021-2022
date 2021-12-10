package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.time.Clock;

@Config
@TeleOp(name="A - Autonomous Simulator (X)")
public class AutonSimulator extends LinearOpMode {
    SampleMecanumDrive drive;
    RevBlinkinLedDriver colorstrip2;

    // Dashboard Configurations
    public static boolean instaStop = false;
    public static double brakingDistance = 5;
    public static double decel = DriveConstants.MAX_ACCEL;
    public static double velMax = DriveConstants.MAX_VEL;
    public static double distNormal = 72;


    double iniTime;

    enum State {
        INIT,
        FORWARD,
        BRAKING,
        RETURN,
        STOPPED
    }

    State state = State.INIT;
    TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(velMax,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH);
TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(decel);
FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        NanoClock clock = NanoClock.system();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        colorstrip2 = hardwareMap.get(RevBlinkinLedDriver.class,"colorstrip");
        colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);

        dashboard = FtcDashboard.getInstance();


        waitForStart();
        new Thread(()->{
            boolean qX = false;
            while(opModeIsActive() && !isStopRequested()) {
                if(gamepad1.x && !qX) {
                    if(state.equals(State.FORWARD)) {
                        colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        drive.trajectorySequenceRunner.cancelTraj();
                    }
                    qX = true;
                }
                else if(!gamepad1.x && qX) {
                    qX = false;
                }
            }
        }).start();
        while(opModeIsActive() && !isStopRequested()) {
            state = State.FORWARD;
            colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
            double iniX = drive.getPoseEstimate().getX();
            iniTime = clock.seconds();
            goForward();
            state = State.BRAKING;
            if(brakingDistance != 0) {
                if(!instaStop) {
                    double distance = drive.getPoseEstimate().getX()-iniX;
                    goSlower(distance);
                }
            }
            state = State.RETURN;
            colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
            goHome();


        }
        state = State.STOPPED;
    }
    public void goForward() {
        TelemetryPacket pack = new TelemetryPacket();

        Trajectory trajForward = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(distNormal,0,0))
                .build();
        drive.followTrajectory(trajForward);
    }
    public void goSlower(double distance) {
        Trajectory trajForward = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(brakingDistance,velocityConstraint,accelerationConstraint)
                .build();
        drive.followTrajectoryTime(trajForward,iniTime);
    }
    public void goHome() {
        Trajectory backHome = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d())
                .build();
        drive.followTrajectory(backHome);
    }
}
