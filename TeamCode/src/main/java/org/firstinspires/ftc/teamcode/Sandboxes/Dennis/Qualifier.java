package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    public static Pose2d duckPoseOne = new Pose2d(-60.5, 63.4, Math.toRadians(45));
    public static Pose2d duckPoseTwo = new Pose2d(-60.5, 63.4, Math.toRadians(46.398));
    public static Pose2d duckPoseThree = new Pose2d(-60.5, 63.4, Math.toRadians(36.28));



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

        duckSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        /**
         * Movement Trajectory
         */
        Trajectory duckTrajOne = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(duckPoseOne)
                .build();

        Trajectory duckTrajTwo = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(duckPoseTwo)
                .build();

        Trajectory duckTrajThree = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(duckPoseThree)
                .build();
        /** end of trajectories */

        waitForStart();


        drive.followTrajectory(duckTrajOne);
        spinDuck();
        //goHome(drive.getPoseEstimate());
        //drive.followTrajectory(duckTrajTwo);
        //spinDuck();
        //goHome(drive.getPoseEstimate());
        //drive.followTrajectory(duckTrajThree);
        //spinDuck();
        //goHome(drive.getPoseEstimate());


        // spin

        // end spin

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

    public static void goHome(Pose2d currentPose) {
        Trajectory returnHome = drive.trajectoryBuilder(currentPose)
                .lineToSplineHeading(startPose)
                .build();
        drive.followTrajectory(returnHome);
    }

    public static void spinDuck() {
        duckSpinner.setPower(.33);
        try {
            Thread.sleep(2500);
        } catch (Exception ex) { return; }
        duckSpinner.setPower(0);
    }
}

