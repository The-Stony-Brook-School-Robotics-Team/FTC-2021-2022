package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.BearsUtil.T265Controller;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class ARR265PathingTeleOp2 extends LinearOpMode {
    SampleMecanumDrive drive;
    volatile AutonomousStates2 state = AutonomousStates2.STOPPED;
    boolean qA, qB, qX, qY;
    Object stateMutex = new Object();
    Pose2d loc2;
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
            synchronized (stateMutex) {telemetry.addData("State",state);}
            telemetry.update();

            }

        }).start();
        synchronized (stateMutex) {state = AutonomousStates2.One_SPLINE;}
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
        T265Controller.shutDown();

    }

    public void doRobotStateAction()
    {
        switch(state) {
            case STOPPED:
                return;
            case One_SPLINE:
                drive.setPoseEstimate(new Pose2d(0,65,-Math.PI/2));
                Pose2d loc = drive.getPoseEstimate();
                Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0, 65, -Math.PI / 2), false)
                        //.splineTo(new Vector2d(loc.getX()-24*Math.cos(loc.getHeading()),loc.getY()+24*Math.sin(loc.getHeading())),loc.getHeading()-Math.PI/4)
                        .splineToSplineHeading(new Pose2d(24, 65, 0), 0)
                        .splineToSplineHeading(new Pose2d(43, 48, 0), -Math.PI / 4)
                        .splineToSplineHeading(new Pose2d(65, 24, 0), -Math.PI / 2)
                        .splineToSplineHeading(new Pose2d(65, 12, 0), -Math.PI / 2)
                        .build();
                drive.followTrajectory(traj1);
                drive.update();
                synchronized (stateMutex) {
                    state = AutonomousStates2.STOPPED;
                }
            /*case Two_SPLINE2:
                Pose2d loc2 = drive.getPoseEstimate();
                Trajectory traj2 = drive.trajectoryBuilder(loc2)
                        //.splineTo(new Vector2d(loc2.getX()+24*Math.cos(loc2.getHeading()),loc2.getY()+24*Math.sin(loc2.getHeading())),loc2.getHeading()-Math.PI/4)
                        .splineTo(new Vector2d(loc2.getX()+40, loc2.getY()+40), loc2.getHeading())
                        .build();
                drive.followTrajectory(traj2);
                drive.update();
                synchronized (stateMutex) {
                    state = AutonomousStates2.STOPPED;
                }
*/
        }
    }


}

enum AutonomousStates2 {
   STOPPED,
   GAMEPAD,
   One_SPLINE,
    Two_SPLINE2

}