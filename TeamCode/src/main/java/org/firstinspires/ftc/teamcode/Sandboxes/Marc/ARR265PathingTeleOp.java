package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import static org.firstinspires.ftc.teamcode.BearsUtil.MotorEncoderController.motorNames;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
@Config
public class ARR265PathingTeleOp extends LinearOpMode {
    SampleMecanumDrive drive;
    RobotState state = RobotState.STOPPED;
    boolean qA, qB, qX, qY;
    public static int DX = 20;
    public static int DY = 20;
    public static int DH = 90;

    @Override
    public void runOpMode() throws InterruptedException {


        drive= new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(2000);
        waitForStart();
        new Thread(()->{
            while(opModeIsActive()) {
                Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            }

        }).start();
        while(opModeIsActive()) {


            if(gamepad1.a && !qA) {
                qA = true;
                state = RobotState.AUTO;
            }
            else if (!gamepad1.a && qA) {
                qA = false;
            }
            if(gamepad1.b && !qB) {
                qB = true;
                state = (state != RobotState.STOPPED) ? RobotState.STOPPED : RobotState.GAMEPAD;
            }
            else if (!gamepad1.b && qB) {
                qB = false;
            }
           doRobotStateAction();
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();


        }

    }

    public void doRobotStateAction()
    {
        switch(state) {
            case STOPPED:
                return;
            case AUTO:
                Pose2d loc = drive.getPoseEstimate();
                Trajectory traj = drive.trajectoryBuilder(loc)
                        .lineToSplineHeading(new Pose2d(loc.getX() + DX,loc.getY() + DY,loc.getHeading() + Math.toRadians(DH)))
                        .build();

                drive.followTrajectory(traj);
                drive.update();
                state = RobotState.STOPPED;
            case GAMEPAD:
                state = RobotState.STOPPED;
        }
    }


}
