package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;


@TeleOp(name = "P - Winston Movement2", group = "default")
public class WinstonMovementTest2 extends LinearOpMode {

    private static Pose2d homePose = new Pose2d(0, 0,0);
    // Dashboard
    private static FtcDashboard dashboard = FtcDashboard.getInstance();
    // Drive
    private static SampleMecanumDrive drive;
    // The coolest man alive variable
    private static int x = 48;


    public static Pose2d toPos = new Pose2d(48,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0,0));
        waitForStart();

        while(!isStopRequested()) {
            TelemetryPacket telemPacket = new TelemetryPacket();
            Canvas ftcField = telemPacket.fieldOverlay();
            DashboardUtil.drawRobot(ftcField, drive.getPoseEstimate());
            dashboard.sendTelemetryPacket(telemPacket);


            if(x < 53) {
                Trajectory mainTraj1 = drive.trajectoryBuilder(homePose)
                        .lineToSplineHeading(toPos)
                        .build();

                Trajectory mainTraj2 = drive.trajectoryBuilder(toPos)
                        .lineToSplineHeading(homePose)
                        .build();

                drive.followTrajectory(mainTraj1);
                drive.followTrajectory(mainTraj2);
            } else {

                Trajectory mainTraj3 = drive.trajectoryBuilder(homePose)
                        .lineToSplineHeading(new Pose2d(40, 0, 0))
                        .build();


                drive.followTrajectory(mainTraj3);
                stop();
            }

            x++;
            toPos = new Pose2d(x,0,0);
            telemetry.addData("x: ", x);
            telemetry.update();
        }


    }
}
