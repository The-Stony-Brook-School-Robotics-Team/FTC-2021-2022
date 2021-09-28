package org.firstinspires.ftc.teamcode.drive.Sandboxes.Marc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.BearsUtil.MotorEncoderController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class RoadrunnerTest extends OpMode {

    SampleMecanumDrive drive;
    private boolean qA = false;
    private boolean qB = false;
    private boolean qX = false;
    private boolean qY = false;


    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -0.3*gamepad1.left_stick_y,
                        -0.3*gamepad1.left_stick_x,
                        -0.3*gamepad1.right_stick_x
                )
        );
    }
}
