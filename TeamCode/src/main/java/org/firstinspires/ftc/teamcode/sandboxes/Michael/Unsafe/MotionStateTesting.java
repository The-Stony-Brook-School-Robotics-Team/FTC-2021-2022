package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;


import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstantsMain.kV;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Objects;

@TeleOp
public class MotionStateTesting extends OpMode {

    private SampleMecanumDrive drive;
    private NanoClock clock;
    private MotionProfile activeProfile;
    private double profileStart;
    private boolean movingForwards = true;

    @Override
    public void init() {
        clock = NanoClock.system();
        drive = new SampleMecanumDrive(hardwareMap);

    }

    @Override
    public void start(){
        activeProfile = generateProfile(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getX() + 5, true);
        profileStart = clock.seconds();
    }

    @Override
    public void loop() {
        double profileTime = clock.seconds() - profileStart;


        if (profileTime > activeProfile.duration()) { //if end of path
            // generate a new profile

            activeProfile = generateProfile(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getX(), true);
            profileStart = clock.seconds();
        }

        MotionState motionState = activeProfile.get(profileTime); //where should i be at this time
        double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);

        drive.setDrivePower(new Pose2d(targetPower, 0, 0));
        drive.updatePoseEstimate();

        Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
        double currentVelo = poseVelo.getX();

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_x ,
                        gamepad1.left_stick_y,
                        -gamepad1.right_stick_x
                )
        );
        drive.update();
    }


    private static MotionProfile generateProfile(double startPos, double endPos, boolean movingForward) {
        MotionState start = new MotionState(movingForward ? startPos : endPos, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? endPos : startPos, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
    }
}
