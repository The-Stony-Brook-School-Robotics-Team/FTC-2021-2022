package org.firstinspires.ftc.teamcode.sandboxes.Dennis;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp
public class CustomPidTuning extends OpMode {

    // State Machine States
    private static enum states { REST, MOVING, PROFILING }

    // Default Start State
    private static states currentState = states.REST;

    // Button Functions
    private static boolean pA, pB, pX, pY = false;

    // Main Trajectory
    private static Trajectory trajectoryForward;

    // Drive
    private static SampleMecanumDrive drive;

    // PID + Distance
    public static double DISTANCE = 0;
    public static double kP = 0; // first
    public static double kI = 0; // third / last
    public static double kD = 0; // second

    @Override
    public void init() {
        // Setup RoadRunner
        drive = new SampleMecanumDrive(hardwareMap);

        // Init Trajectory
        trajectoryForward = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(DISTANCE)
                .build();
    }

    @Override
    public void loop() {
        switch(currentState) {




        }



    }
}
