package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

public class TankTrajectoryTest extends OpMode {

    private SampleTankDrive drive;
    private DcMotor[] left;
    private DcMotor[] right;
    private Trajectory testTraj;
    @Override
    public void init() {
        drive = new SampleTankDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        testTraj = new TrajectoryBuilder(drive.getPoseEstimate(), SampleTankDrive.VEL_CONSTRAINT, SampleTankDrive.accelConstraint)
                .splineTo(new Vector2d(15, 15), Math.toRadians(0))
                .build();
    }

    @Override
    public void loop() {
        drive.setMotorPowers(-gamepad1.left_stick_y + gamepad1.right_stick_x, -gamepad1.left_stick_y - gamepad1.right_stick_x);
        if(gamepad1.a){
            drive.followTrajectory(testTraj);
        }
    }
}
