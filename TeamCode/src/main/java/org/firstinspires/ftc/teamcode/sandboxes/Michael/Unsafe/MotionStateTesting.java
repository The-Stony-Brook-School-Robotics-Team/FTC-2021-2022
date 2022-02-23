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
    private String state;
    private double endPos;
    private double startPos;
    private double endPath = 1;

    @Override
    public void init() {
        clock = NanoClock.system();
        drive = new SampleMecanumDrive(hardwareMap);
        state = "Init";
        startPos = drive.getPoseEstimate().getX();
        endPos = drive.getPoseEstimate().getX() + 25;

    }

    @Override
    public void start(){
        activeProfile = generateProfile(startPos, endPos, movingForwards);
        profileStart = clock.seconds();
    }

    @Override
    public void loop() {
        double profileTime = clock.seconds() - profileStart;


        if (profileTime > activeProfile.duration() && gamepad1.a) { //if end of path
            state = "End of Path";
            // generate a new profile
           // movingForwards = !movingForwards;
            activeProfile = generateProfile(startPos, endPos, movingForwards);
            profileStart = clock.seconds();
        }
        if(profileTime > activeProfile.duration()){
            endPath = 0;
        }
        else{endPath = 1;}

        MotionState motionState = activeProfile.get(profileTime); //where should i be at this time
        double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);

        drive.setDrivePower(new Pose2d(targetPower*endPath, 0, 0));
        drive.updatePoseEstimate();
        
        drive.setDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_x ,
                        gamepad1.left_stick_y,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();
        telemetry.addData("State: ", state);
        telemetry.addData("Time: ", profileTime);
        telemetry.update();
    }


    private static MotionProfile generateProfile(double startPos, double endPos, boolean movingForward) {
        MotionState start = new MotionState(movingForward ? startPos : endPos, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? endPos : startPos, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal, MAX_VEL, MAX_ACCEL);
    }
}
