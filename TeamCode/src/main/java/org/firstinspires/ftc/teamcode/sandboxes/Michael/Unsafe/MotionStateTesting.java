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
    private MotionProfileWithVector profileOne;

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
        activeProfile = generateProfile(startPos, endPos);

        profileOne = generateProfileWithVector(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getX() + 25, VECTORS.X);
        profileStart = clock.seconds();
    }

    @Override
    public void loop() {
        doMotionPath();
        telemetry.update();
    }


    private static MotionProfile generateProfile(double startPos, double endPos) {
        MotionState start = new MotionState(startPos, 0, 0, 0);
        MotionState end = new MotionState(endPos, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, end, MAX_VEL, MAX_ACCEL);
    }

    private static MotionProfileWithVector generateProfileWithVector(double startPos, double endPos, VECTORS v){
        return new MotionProfileWithVector(generateProfile(startPos, endPos), v);
    }


    public void doMotionPath(MotionProfileWithVector ... motionProfiles){
        for (MotionProfileWithVector currentProfile: motionProfiles) {
            VECTORS v = currentProfile.getVector();
            double profileStart = clock.seconds();
            double profileTime = clock.seconds() - profileStart;
            while(profileTime > currentProfile.getMotionProfile().duration()){
                profileTime = clock.seconds() - profileStart;
                MotionState motionState = currentProfile.getMotionProfile().get(profileTime); //where should i be at this time
                double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);
                switch (v){
                    case X:
                        drive.setDrivePower(new Pose2d(targetPower, 0, 0));
                }
                
                drive.updatePoseEstimate();
            }
        }
        drive.setDrivePower(new Pose2d(0, 0, 0));
        drive.updatePoseEstimate();
    }

    public enum VECTORS{
        X,
        Y,
        HEADING
    }

    public static class MotionProfileWithVector{
        private MotionProfile motionProfile;
        private VECTORS vector;
        public MotionProfileWithVector(MotionProfile profile, VECTORS v){
            motionProfile = profile;
            vector = v;
        }

        public MotionProfile getMotionProfile() {
            return motionProfile;
        }
        public VECTORS getVector() {
            return  vector;
        }
    }
}
