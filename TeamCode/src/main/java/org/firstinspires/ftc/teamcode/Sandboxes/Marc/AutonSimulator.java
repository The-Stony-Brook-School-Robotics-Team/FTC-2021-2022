package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="A - Autonomous Simulator (X)")
public class AutonSimulator extends LinearOpMode {
    SampleMecanumDrive drive;
    RevBlinkinLedDriver colorstrip2;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        colorstrip2 = hardwareMap.get(RevBlinkinLedDriver.class,"colorstrip");
        colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
        waitForStart();
        new Thread(()->{
            boolean qX = false;
            while(opModeIsActive() && !isStopRequested()) {
                if(gamepad1.x && !qX) {
                    colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    drive.trajectorySequenceRunner.cancelTraj();
                    qX = true;
                }
                else if(!gamepad1.x && qX) {
                    qX = false;
                }
            }
        }).start();
        while(opModeIsActive() && !isStopRequested()) {
            colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
            goForward();
            goSlower();
            colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
            goHome();


        }
    }
    public void goForward() {
        Trajectory trajForward = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(72,0,0))
                .build();
        drive.followTrajectory(trajForward);
    }
    public void goSlower() {
        Trajectory trajForward = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(5)
                .build();
        drive.followTrajectory(trajForward);
    }
    public void goHome() {
        Trajectory backHome = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d())
                .build();
        drive.followTrajectory(backHome);
    }
}
