package org.firstinspires.ftc.teamcode.sandboxes.William.MovementControl;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

//@Autonomous(name = "----TankTank", group = "Tank")
public class ___TankTank extends LinearOpMode {

    public static double DISTANCE = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            Trajectory trajectoryForward = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(DISTANCE)
                    .build();

            drive.followTrajectory(trajectoryForward);
            Trajectory trajectoryBackward = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(DISTANCE)
                    .build();
            drive.followTrajectory(trajectoryBackward);
        }
    }
}
