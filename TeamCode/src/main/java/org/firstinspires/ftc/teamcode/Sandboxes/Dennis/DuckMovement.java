package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.misc.Converter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class DuckMovement extends LinearOpMode {

    private static SampleMecanumDrive drive;
    private static DcMotor duckSpinner;
    private static RevBlinkinLedDriver rgb;

    /**
     * Duck Spinner Movement
     */
    private static Pose2d startPose = new Pose2d(-43, 55);
    private static Pose2d duckPose = new Pose2d(-52, 50, Math.toRadians(40));
    private static Pose2d blueDropOff = new Pose2d(-12.5, 65, 0);
    /**
     * Factory Movement
     */
    // private static Pose2d factoryPose1 = new Pose2d(37, 65, 0);
    // private static Pose2d factoryPose2 = new Pose2d(37, 40, -90);
    // private static Pose2d factoryPose3 = new Pose2d(65, 40, -90);
    // private static Pose2d factoryPose4 = new Pose2d(65, 40, -90);


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

        /**
         * Movement Trajectory
         */
        Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(duckPose)
                .addDisplacementMarker(() -> {
                    /**
                     * Start the duck spinner
                     */
                    duckSpinner.setPower(1);
                    try {
                        Thread.sleep(5000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    /**
                     * Stop the duck spinner
                     */
                    duckSpinner.setPower(0);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                })
                .lineToSplineHeading(blueDropOff)
                /**
                 * Wait 5 seconds cause why not TODO: LOLLOP
                 */
                .addDisplacementMarker(() -> {
                    try {
                        Thread.sleep(5000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                })
                /**
                 * Yeah Yeah Yeah Yeah Yeah
                 */
                .build();

        /**
         * Set Pattern To Ready
         */
        rgb.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);

        /**
         * "Pre-Op-Mode"
         */
        waitForStart();

        /**
         * Set the scanner color
         */
        rgb.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
        /**
         * Run Trajectory
         */
        drive.followTrajectory(traj);
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

