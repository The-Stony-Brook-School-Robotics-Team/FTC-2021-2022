package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;


/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(name = "tank wooh", group = "drive")
public class TestLocalization extends OpMode {
    SampleTankDrive drive;
    NewSlideController newSlideController;
    boolean pressingA = false;
    boolean pressingB = false;
    @Override
    public void init() {
        drive = new SampleTankDrive(hardwareMap);
        newSlideController = new NewSlideController(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            newSlideController.extend(SlideConstants.slideMotorPosition_THREE_DEPOSIT);
        }
        if(gamepad1.b){
            newSlideController.retract();
        }
        drive.setMotorPowers(-gamepad1.left_stick_y + gamepad1.right_stick_x, -gamepad1.left_stick_y - gamepad1.right_stick_x);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("button a", pressingA);
        telemetry.addData("button b", pressingB);
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }
    @Override
    public void stop(){
        newSlideController.killThreads();
    }
}