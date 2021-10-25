package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

import static org.firstinspires.ftc.teamcode.BearsUtil.MotorEncoderController.motorNames;
import static org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe.RobotConfig.CENTER_WHEEL_OFFSET;
import static org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe.RobotConfig.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe.RobotConfig.TRACKWIDTH;
import static org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe.RobotConfig.WHEEL_DIAMETER;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.BearsUtil.T265Controller;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous

public class rrODOM extends LinearOpMode {
    SampleMecanumDrive drive;
    volatile AutonomousStates state = AutonomousStates.STOPPED;
    boolean qA, qB, qX, qY;
    Object stateMutex = new Object();
    Pose2d loc2;
    private static final double TICKS_TO_INCHES = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    private MotorEx encoderLeft, encoderRight, encoderPerp;
    private OdometrySubsystem odometry;
    @Override
    public void runOpMode() throws InterruptedException {
        encoderLeft = new MotorEx(hardwareMap, "leftodom");
        encoderRight = new MotorEx(hardwareMap, "rightodom");
        encoderPerp = new MotorEx(hardwareMap, "backodom");
        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        encoderRight.setInverted(true);

        encoderLeft.resetEncoder();
        encoderRight.resetEncoder();
        encoderPerp.resetEncoder();
        HolonomicOdometry holOdom = new HolonomicOdometry(
                () -> (encoderLeft.getCurrentPosition() * TICKS_TO_INCHES),
                () -> (encoderRight.getCurrentPosition() * TICKS_TO_INCHES), //-
                () -> (encoderPerp.getCurrentPosition() * TICKS_TO_INCHES),
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        odometry = new OdometrySubsystem(holOdom);

        drive= new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Thread.sleep(2000);
        waitForStart();
        new Thread(()->{
            while(opModeIsActive()) {
                //TODO: handler

                //Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                synchronized (stateMutex) {telemetry.addData("State",state);}
                telemetry.update();

            }

        }).start();
        synchronized (stateMutex) {state = AutonomousStates.One_STAGE_ADVANCE;}
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
            case One_STAGE_ADVANCE:
                Pose2d loc = drive.getPoseEstimate();
                Trajectory traj1 = drive.trajectoryBuilder(loc)
                        .lineToSplineHeading(new Pose2d(loc.getX() + 6,loc.getY(),loc.getHeading()))
                        .build();
                drive.followTrajectory(traj1);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.Two_ROTATE;}
            case Two_ROTATE:
                loc2 = drive.getPoseEstimate();
                drive.turn(Math.toRadians(90));
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.Three_BACK_TO_WALL;}
            /*case Twobis_FIXPOS: // skipped
                Trajectory traj2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(loc2.getX(),loc2.getY(),loc2.getHeading() + Math.toRadians(90)))
                        .build();
                drive.followTrajectory(traj2);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.Three_BACK_TO_WALL;}*/
            case Three_BACK_TO_WALL:
                Pose2d loc3 = drive.getPoseEstimate();
                Trajectory traj3 = drive.trajectoryBuilder(loc3)
                        .lineToSplineHeading(new Pose2d(loc3.getX() - 8,loc3.getY(),loc2.getHeading()+Math.toRadians(90)))
                        .build();
                drive.followTrajectory(traj3);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.FORWARD1;}
            case FORWARD1:
                Pose2d loc4 = drive.getPoseEstimate();
                Trajectory traj4 = drive.trajectoryBuilder(loc4)
                        .lineToSplineHeading(new Pose2d(loc4.getX(),loc4.getY()+24,loc4.getHeading()))
                        .build();
                drive.followTrajectory(traj4);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.BACKWARD1;}
            case BACKWARD1:
                Pose2d loc5 = drive.getPoseEstimate();
                Trajectory traj5 = drive.trajectoryBuilder(loc5)
                        .lineToSplineHeading(new Pose2d(loc5.getX(),loc5.getY()-24,loc5.getHeading()))
                        .build();
                drive.followTrajectory(traj5);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.FORWARD3;}
            case FORWARD3:
                Pose2d loc6 = drive.getPoseEstimate();
                Trajectory traj6 = drive.trajectoryBuilder(loc6)
                        .lineToSplineHeading(new Pose2d(loc6.getX(),loc6.getY()+24,loc6.getHeading()))
                        .build();
                drive.followTrajectory(traj6);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.BACKWARD3;}
            case BACKWARD3:
                Pose2d loc7 = drive.getPoseEstimate();
                Trajectory traj7 = drive.trajectoryBuilder(loc7)
                        .lineToSplineHeading(new Pose2d(loc7.getX(),loc7.getY()-24,loc7.getHeading()))
                        .build();
                drive.followTrajectory(traj7);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.FORWARD4;}
            case FORWARD4:
                Pose2d loc8 = drive.getPoseEstimate();
                Trajectory traj8 = drive.trajectoryBuilder(loc8)
                        .lineToSplineHeading(new Pose2d(loc8.getX(),loc8.getY()+24,loc8.getHeading()))
                        .build();
                drive.followTrajectory(traj8);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.BACKWARD4;}
            case BACKWARD4:
                Pose2d loc9 = drive.getPoseEstimate();
                Trajectory traj9 = drive.trajectoryBuilder(loc9)
                        .lineToSplineHeading(new Pose2d(loc9.getX(),loc9.getY()-24,loc9.getHeading()))
                        .build();
                drive.followTrajectory(traj9);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.FORWARD5;}
            case FORWARD5:
                Pose2d loc10 = drive.getPoseEstimate();
                Trajectory traj10 = drive.trajectoryBuilder(loc10)
                        .lineToSplineHeading(new Pose2d(loc10.getX(),loc10.getY()+24,loc10.getHeading()))
                        .build();
                drive.followTrajectory(traj10);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.BACKWARD5;}
            case BACKWARD5:
                Pose2d loc11 = drive.getPoseEstimate();
                Trajectory traj11 = drive.trajectoryBuilder(loc11)
                        .lineToSplineHeading(new Pose2d(loc11.getX(),loc11.getY()-24,loc11.getHeading()))
                        .build();
                drive.followTrajectory(traj11);
                drive.update();
                synchronized (stateMutex) {state = AutonomousStates.STOPPED;}
        }
    }


}
enum AutonomousStates {
    STOPPED(0),
    GAMEPAD(100),
    One_STAGE_ADVANCE(1),
    Two_ROTATE(2),
    //Twobis_FIXPOS(2.5),
    Three_BACK_TO_WALL(3),
    FORWARD1(4),
    BACKWARD1(5),
    FORWARD2(6),
    BACKWARD2(7),
    FORWARD3(8),
    BACKWARD3(9),
    FORWARD4(10),
    BACKWARD4(11),
    FORWARD5(12),
    BACKWARD5(13);

    public double home = 0;
    AutonomousStates(double i) {
        home = i;
    }
    public double getID(){
        return home;
    }
}