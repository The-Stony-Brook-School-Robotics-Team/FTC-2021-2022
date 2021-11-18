package org.firstinspires.ftc.teamcode.common.tuning.custom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.custom.CustomPIDTunerDrive;

@Config
@Autonomous(group = "drive", name="T - BackAndForthInfiniteTimeout")
public class BackAndForthInfiniteTimeout extends LinearOpMode {

    public static double DISTANCE = 48;
    private enum states { STILL, READY, WORKING }
    private states currentState = states.STILL;

    private boolean pA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init
        CustomPIDTunerDrive drive = new CustomPIDTunerDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0));

        currentState = states.READY;
        telemetry.addData("Status: ", currentState);

        // On Start
        waitForStart();

        telemetry.clearAll();

        // Work Loop
        while (opModeIsActive() && !isStopRequested()) {
            if(gamepad1.a && !pA) {
                pA = true;
            } else {
                currentState = states.WORKING;
                telemetry.addData("Status: ", currentState);
                pA = false;
            }

            if(currentState == states.WORKING) {
                Vector2d targetVec = new Vector2d(DISTANCE, 0);
                Trajectory targetTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineTo(targetVec)
                        .build();

                drive.followTrajectory(targetTraj);
            }

            telemetry.update();
        }
    }
}