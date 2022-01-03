<<<<<<< HEAD
package org.firstinspires.ftc.teamcode.Sandboxes.William.CustomizedMecanumDriveTest;
=======
package org.firstinspires.ftc.teamcode.sandboxes.William.CustomizedMecanumDriveTest;
>>>>>>> new-drivetrain

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Sandboxes.William.Util.CustomizedMecanumDrive;
import org.firstinspires.ftc.teamcode.common.tuning.timeout.TurnTest;
import org.firstinspires.ftc.teamcode.drive.timeout.CustomTimeoutTuningDrive;

@Config
@Autonomous(group = "drive", name = "Customized Trajectory Builder Test")
<<<<<<< HEAD
public class CustomizedTrajectoryBuilderTest extends LinearOpMode{
=======
public class CustomizedTrajectoryBuilderTest extends LinearOpMode {
>>>>>>> new-drivetrain

    @Override
    public void runOpMode() throws InterruptedException {
        // Init
        CustomizedMecanumDrive customizedMecanumDrive = new CustomizedMecanumDrive(hardwareMap);

        // On Start
        waitForStart();

        customizedMecanumDrive.setTimeoutValue_SE(5);

        Trajectory trajectoryForward = customizedMecanumDrive
                .trajectoryBuilder(new Pose2d(-2,-2))
                .build();
        customizedMecanumDrive.followTrajectory(trajectoryForward);
    }
}
