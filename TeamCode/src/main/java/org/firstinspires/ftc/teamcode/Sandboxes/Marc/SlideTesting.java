package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideTarget;
@Config
@TeleOp(name = "A - Backup TeleOp")
public class SlideTesting extends LinearOpMode
{
    public static double incrementValue = 0.05;
    public static double power = 0.2;
    public static int ms = 300;
    boolean pA = false, pUp = false, pDown = false;
    boolean pY = false;
    SlideController slideController;
SampleMecanumDrive drive;
IntakeControllerBlue bu;
IntakeControllerRed red;
    boolean slideOut = false;
    private boolean qX;
    private boolean pB;
    private boolean pRB;

    @Override
    public void runOpMode() throws InterruptedException {
        slideController = new SlideController(hardwareMap, telemetry);
drive = new SampleMecanumDrive(hardwareMap);
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(30, 2,DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);
        bu = new IntakeControllerBlue(hardwareMap,telemetry);
        red = new IntakeControllerRed(hardwareMap,telemetry);


        //bu.setState(IntakeState.PARK);
//        red.setState(IntakeState.PARK);

        //slideController.initTeleop();
        //slideController.targetParams = SlideTarget.TOP_DEPOSIT;

        waitForStart();

        while(!isStopRequested()) {
            if(gamepad1.y && !pY) {
                slideController.retractSlide();
                slideController.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideController.slideMotor.setPower(-power);
                Thread.sleep(ms);
                slideController.slideMotor.setPower(0);
                slideController.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideController.slideMotor.setTargetPosition(slideController.slideMotor.getCurrentPosition());
                slideController.slideMotor.setPower(.9);
                pY = true;
            } else if(!gamepad1.y && pY) {
                pY = false;
            }

            //slideController.collectCapstone();

            if(gamepad1.right_bumper && !pRB) {
                slideController.collectCapstone();
                pRB = true;
            } else if(!gamepad1.right_bumper && pRB) {
                pRB = false;
            }

            if(gamepad1.a && !pA) {
                slideController.dropCube();
                pA = true;
            } else if(!gamepad1.a && pA) {
                pA = false;
            }
            if(gamepad1.b && !pB) {
                slideController.extendSlide();
                pB = true;
            } else if(!gamepad1.b && pB) {
                pB = false;
            }

            if(gamepad1.right_stick_y < -0.02 || gamepad1.right_stick_y > -0.02) {
                //drive.setWeightedDrivePower(new Pose2d());
                double stickValue = gamepad1.right_stick_y * -0.2;
                slideController.incrementEncoderPosition((int) (stickValue * Configuration.DefaultSlideTicks));
                //Log.d("SLIDE TESTER", "Current Slide Motor Ticks: " + slideController.getSlideMotorPosition());
            }

            if(gamepad1.dpad_up && !pUp) {
                drive.setWeightedDrivePower(new Pose2d());
                slideController.incrementVerticalServo(incrementValue);
                //Log.d("SLIDE TESTER", "Current Slide Servo Position: " + slideController.getVerticalServoPosition());
                pUp = true;
            } else if(!gamepad1.dpad_up && pUp) {
                pUp = false;
            }

            if(gamepad1.dpad_down && !pDown) {
                drive.setWeightedDrivePower(new Pose2d());
                slideController.incrementVerticalServo(-incrementValue);
                //Log.d("SLIDE TESTER", "Current Slide Servo Position: " + slideController.getVerticalServoPosition());
                pDown = true;
            } else if(!gamepad1.dpad_down && pDown) {
                pDown = false;
            }
            if(gamepad1.x && !qX) {
                qX = true;
                slideController.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                slideController.slideMotor.setPower(-power);
                Thread.sleep(ms);
                slideController.slideMotor.setPower(0);
                slideController.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideController.slideMotor.setTargetPosition(slideController.slideMotor.getCurrentPosition());
                slideController.slideMotor.setPower(.9);

            } else if(!gamepad1.x && qX) {
                qX = false;
            }

            telemetry.addData("Slide Ext",slideController.slideMotor.getCurrentPosition());
            telemetry.addData("Slide Angle",slideController.verticalServo.getPosition());
            telemetry.update();



        }



    }
}
