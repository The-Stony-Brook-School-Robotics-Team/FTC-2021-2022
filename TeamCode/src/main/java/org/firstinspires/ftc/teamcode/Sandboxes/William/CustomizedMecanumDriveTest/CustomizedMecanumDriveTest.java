package org.firstinspires.ftc.teamcode.sandboxes.William.CustomizedMecanumDriveTest;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sandboxes.William.Util.CustomizedMecanumDrive;
import org.firstinspires.ftc.teamcode.sandboxes.William.Util.CustomizedTrajectorySequenceRunner;

@Config
@Autonomous(group = "drive", name = "Customized Mecanum Drive Test")
public class CustomizedMecanumDriveTest extends LinearOpMode {
    private final PIDFController turnController;
    private final PIDCoefficients PIDCoefficient = new PIDCoefficients();

    public CustomizedMecanumDriveTest(PIDFController turnController) {
        this.turnController = turnController;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Init
        CustomizedMecanumDrive customizedMecanumDrive = new CustomizedMecanumDrive(hardwareMap);

        // On Start
        waitForStart();

        customizedMecanumDrive.setTimeoutValue_SE(5);

        Trajectory trajectoryForward = customizedMecanumDrive
                .trajectoryBuilder(customizedMecanumDrive.getPoseEstimate())
                .forward(48)
                .build();

        Thread stopTrajectory = new Thread(new Runnable() {
            NanoClock clock = NanoClock.system();
            final double startTime = clock.seconds();

            @Override
            public void run() {
                while (true)
                    if (clock.seconds() - startTime > 5.0) {
                        CustomizedTrajectorySequenceRunner.needEmergencyStop = true;
                        break;
                    }
            }
        });

        stopTrajectory.start();
        customizedMecanumDrive.followTrajectory(trajectoryForward);
    }
}
