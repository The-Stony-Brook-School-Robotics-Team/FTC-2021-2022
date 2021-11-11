package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class PathingTest extends LinearOpMode {
    SampleMecanumDrive drive;
    volatile AutonomousStates3 state = AutonomousStates3.STOPPED;
    boolean qA, qB, qX, qY;
    Object stateMutex = new Object();
    Pose2d loc2;
    @Override
    public void runOpMode() throws InterruptedException {


        drive= new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(2000);
        waitForStart();
        drive.setPoseEstimate(new Pose2d());
        new Thread(()->{
            while(opModeIsActive()) {
                Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.addData("version", 6);
            synchronized (stateMutex) {telemetry.addData("State",state);}
            telemetry.update();

            }

        }).start();
        synchronized (stateMutex) {state = AutonomousStates3.One_FORWARD1;}
        while(opModeIsActive() && !isStopRequested()) {


            if(gamepad1.a && !qA) {
                qA = true;
            }
            else if (!gamepad1.a && qA) {
                qA = false;
            }

            doRobotStateAction();
            /*drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );*/
            drive.update();


        }

    }

    public void doRobotStateAction()
    {
        switch(state) {
            case STOPPED:
                return;
            case One_FORWARD1:
                Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(), false)
                        .lineToSplineHeading(new Pose2d(72,0,0))
                        .build();
                drive.followTrajectory(traj1);
                drive.update();
                synchronized (stateMutex) {
                    state = AutonomousStates3.Two_TURN1;
                }
            case Two_TURN1:
                drive.turn(-Math.PI/2);
                drive.update();
                synchronized (stateMutex) {
                    state = AutonomousStates3.Three_FORWARD2;
                }
            case Three_FORWARD2:
                Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(72,0,-Math.PI/2), false)
                        .lineToSplineHeading(new Pose2d(72,-24,-Math.PI/2))
                        .build();
                drive.followTrajectory(traj3);
                drive.update();
                synchronized (stateMutex) {
                    state = AutonomousStates3.Four_TURN2;
                }
            case Four_TURN2:
                drive.turn(-Math.PI/2);
                drive.update();
                synchronized (stateMutex) {
                    state = AutonomousStates3.Five_FORWARD3;
                }
            case Five_FORWARD3:
                Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(72,-24,-Math.PI), false)
                        .lineToSplineHeading(new Pose2d(0,-24,-Math.PI))
                        .build();
                drive.followTrajectory(traj5);
                drive.update();
                synchronized (stateMutex) {
                    state = AutonomousStates3.STOPPED;
                }
        }
    }


}

enum AutonomousStates3 {
   STOPPED,
   GAMEPAD,
   One_FORWARD1,
    Two_TURN1,
    Three_FORWARD2,
    Four_TURN2,
    Five_FORWARD3

}