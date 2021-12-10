<<<<<<< Updated upstream
package org.firstinspires.ftc.teamcode.sandboxes.William.Tuning;
=======
package org.firstinspires.ftc.teamcode.sandboxes.William.CustomizedMecanumDriveTest;
>>>>>>> Stashed changes

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sandboxes.William.Util.CustomizedMecanumDrive;
<<<<<<< Updated upstream
=======
import org.firstinspires.ftc.teamcode.common.tuning.timeout.TurnTest;
import org.firstinspires.ftc.teamcode.drive.timeout.CustomTimeoutTuningDrive;
>>>>>>> Stashed changes

@Config
@Autonomous(group = "drive", name = "Customized Mecanum Drive Test")
public class CustomizedMecanumDriveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Init
        CustomizedMecanumDrive customizedMecanumDrive = new CustomizedMecanumDrive(hardwareMap);

        // On Start
        waitForStart();

        customizedMecanumDrive.setTimeoutValue_SE(5);
        Trajectory trajectoryForward = customizedMecanumDrive.trajectoryBuilder(customizedMecanumDrive.getPoseEstimate())
                .forward(24)
                .build();
        customizedMecanumDrive.followTrajectory(trajectoryForward);
    }
}
