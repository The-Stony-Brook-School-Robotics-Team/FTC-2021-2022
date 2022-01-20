package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideController;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideTarget;

@TeleOp(name = "AAAAA - COOL TELEOP TESTER")
public class SlideTesting extends LinearOpMode
{
    boolean pA = false, pUp = false, pDown = false;
    SlideController slideController;
SampleMecanumDrive drive;
IntakeControllerBlue bu;
IntakeControllerRed red;
    boolean slideOut = false;
    private boolean qX;

    @Override
    public void runOpMode() throws InterruptedException {
        slideController = new SlideController(hardwareMap, telemetry);
drive = new SampleMecanumDrive(hardwareMap);
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(30, 2,DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);
        bu = new IntakeControllerBlue(hardwareMap,telemetry);
        red = new IntakeControllerRed(hardwareMap,telemetry);

        bu.setState(IntakeState.PARK);
        red.setState(IntakeState.PARK);
        while(!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            if(gamepad1.a && !pA) {
                //drive.setWeightedDrivePower(new Pose2d());
                double angle= Math.toRadians(51);
                double sideOfRobot = 6;
                drive.setPoseEstimate(new Pose2d(14,65.5,0));
                //drive.setWeightedDrivePower(new Pose2d()); // stop robot
                Pose2d currentPos = drive.getPoseEstimate();
                //Pose2d targetPos = new Pose2d(currentPos.getX()-(sideOfRobot*Math.cos(angle)+sideOfRobot*Math.sin(angle)),currentPos.getY()-(sideOfRobot*Math.sin(angle)-sideOfRobot*Math.cos(angle)),currentPos.getHeading() - angle);
                Pose2d target = new Pose2d(5.58,64.47,-Math.toRadians(52));
                drive.followTrajectory(drive.trajectoryBuilder(currentPos)
                        .lineToSplineHeading(target,velocityConstraint,accelerationConstraint)
                        .build());
                /*   if(slideOut) {
                    slideController.retractSlide();
                    slideOut = true;
                } else if(!slideOut) {
                    slideController.extendSlide();
                    slideOut = false;
                }*/
                pA = true;
            } else if(!gamepad1.a && pA) {
                pA = false;
            }

            if(gamepad1.right_stick_y < -0.02 || gamepad1.right_stick_y > -0.02) {
                //drive.setWeightedDrivePower(new Pose2d());
                double stickValue = gamepad1.right_stick_y * -0.2;
                slideController.incrementEncoderPosition((int) (stickValue * Configuration.DefaultSlideTicks));
                Log.d("SLIDE TESTER", "Current Slide Motor Ticks: " + slideController.getSlideMotorPosition());
            }

            if(gamepad1.dpad_up && !pUp) {
                drive.setWeightedDrivePower(new Pose2d());
                slideController.incrementVerticalServo(0.02);
                Log.d("SLIDE TESTER", "Current Slide Servo Position: " + slideController.getVerticalServoPosition());
                pUp = true;
            } else if(!gamepad1.dpad_up && pUp) {
                pUp = false;
            }

            if(gamepad1.dpad_down && !pDown) {
                drive.setWeightedDrivePower(new Pose2d());
                slideController.incrementVerticalServo(-0.02);
                Log.d("SLIDE TESTER", "Current Slide Servo Position: " + slideController.getVerticalServoPosition());
                pDown = true;
            } else if(!gamepad1.dpad_down && pDown) {
                pDown = false;
            }
            if(gamepad1.x && !qX) {
                drive.setWeightedDrivePower(new Pose2d());
                slideController.extendDropRetract(SlideTarget.THREE_DEPOSIT);
                slideController.extendSlide();
                slideController.dropCube();
                slideController.retractSlide();
                qX = true;
            } else if(!gamepad1.x && qX) {
                qX = false;
            }

            telemetry.addData("Slide Ext",slideController.slideMotor.getCurrentPosition());
            telemetry.addData("Slide Angle",slideController.verticalServo.getPosition());
            telemetry.update();



        }



    }
}
