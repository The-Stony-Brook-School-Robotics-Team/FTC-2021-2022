package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@TeleOp(name="A - Auton Qualifier One", group="default")
public class Qualifier extends LinearOpMode {
    //TODO: ahahha stinky add rest of controllers
    // private IntakeController frontIntake = new IntakeController(hardwareMap, telemetry, IntakeSide.FRONT);



    private static SampleMecanumDrive drive;
    private static DcMotor duckSpinner;
    private static RevBlinkinLedDriver rgb;

    /**
     * Duck Spinner Movement
     */
    public static Pose2d startPose = new Pose2d(-42, 66, 0);
    public static Pose2d duckPose = new Pose2d(-58.75, 62.28, Math.toRadians(74.79));
    public static Pose2d redHubDropoff = new Pose2d(-12.14, 66.52, 0);

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
                .back(12)
                .build();



        rgb.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);

        waitForStart();

        rgb.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);

        // go to spinner
        drive.followTrajectory(traj);
        drive.update();
        // spin
        do {

        } while(WheelControl.signal!=2);




        /**
         * Set the green color
         */
        rgb.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

