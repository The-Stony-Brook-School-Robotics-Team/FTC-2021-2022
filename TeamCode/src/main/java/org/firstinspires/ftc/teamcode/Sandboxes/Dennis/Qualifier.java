package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.misc.Converter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.controllers.*;
import org.sbs.bears.robotframework.enums.*;

@TeleOp(name="AAA - Qualifier One", group="default")
public class Qualifier extends LinearOpMode {
    //TODO: ahahha stinky add rest of controllers
    private IntakeController frontIntake = new IntakeController(hardwareMap, telemetry, IntakeSide.FRONT);



    private static SampleMecanumDrive drive;
    private static DcMotor duckSpinner;
    private static RevBlinkinLedDriver rgb;

    /**
     * Duck Spinner Movement
     */
    private static Pose2d startPose = new Pose2d(-39.24, 51.95, 0);
    private static Pose2d duckPose = new Pose2d(-52.102, 48.591, 1.3487);
    private static Pose2d blueDropOff = new Pose2d(-9.94, 51.812, 0);

    /**
     * Convert Units (Pose2d to Vector2d)
     */
    private static Converter converter = new Converter();


    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * Robot Initialization
         */

        drive = new SampleMecanumDrive(hardwareMap);
        duckSpinner = hardwareMap.get(DcMotor.class, "duck");
        rgb = hardwareMap.get(RevBlinkinLedDriver.class,"rgb");
        drive.setPoseEstimate(startPose);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.update();

        /**
         * Movement Trajectory
         */
        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(duckPose)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .lineToSplineHeading(blueDropOff)
                .build();



        rgb.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);

        waitForStart();

        rgb.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);

        // go to spinner
        drive.followTrajectory(traj);
        drive.update();
        // spin
        duckSpinner.setPower(.3);
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        duckSpinner.setPower(0);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        // follow second
        drive.followTrajectory(traj2);
        drive.update();


        drive.update();
        /**
         * Set the green color
         */
        rgb.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        /**
         * Main Runtime
         */
        while(!isStopRequested()) {
            telemetry.addData("runtime: ", getRuntime());
            telemetry.addData("x: ", drive.getPoseEstimate().getX());
            telemetry.addData("y: ", drive.getPoseEstimate().getY());
            telemetry.addData("heading: ", drive.getPoseEstimate().getHeading());
            telemetry.update();
            drive.update();
        }

    }



}

