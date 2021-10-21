package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import static org.firstinspires.ftc.teamcode.BearsUtil.MotorEncoderController.motorNames;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class ARR265PathingTeleOp extends LinearOpMode {
    SampleMecanumDrive drive;
    RobotState state = RobotState.STOPPED;
    boolean qA, qB, qX, qY;

    @Override
    public void runOpMode() throws InterruptedException {


        drive= new SampleMecanumDrive(hardwareMap);
    waitForStart();

        while(opModeIsActive()) {


            if(gamepad1.a && !qA) {
                qA = true;
                state = RobotState.AUTO;
                continue;
            }
            else if (!gamepad1.a && qA) {
                qA = false;
            }

            doRobotStateAction();



        }

    }

    public void doRobotStateAction()
    {
        switch(state) {
            case STOPPED:
                return;
            case AUTO:
                Trajectory traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(20,20,Math.PI))
                        .build();

                drive.followTrajectoryAsync(traj);
                state = RobotState.STOPPED;
        }
    }


}
