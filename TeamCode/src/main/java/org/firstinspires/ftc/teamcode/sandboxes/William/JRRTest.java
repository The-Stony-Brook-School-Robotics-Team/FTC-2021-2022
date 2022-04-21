package org.firstinspires.ftc.teamcode.sandboxes.William;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.sbs.bears.robotframework.controllers.JRR;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;

//@Config
//@TeleOp(name = "JRRTest", group = "JRR")
public class JRRTest extends LinearOpMode {
    RoadRunnerController jrr;
    public static int forwardDistance = 40;

    @Override
    public void runOpMode() throws InterruptedException {
        jrr =  new RoadRunnerController(hardwareMap, telemetry);
        waitForStart();

        Thread stopJRRThread = new Thread(() -> {
            while (opModeIsActive()) {
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            jrr.stopRobot();
        });

        stopJRRThread.start();

        jrr.setPos(new Pose2d());
        while (opModeIsActive()) {
            jrr.getDrive().followTrajectory(
                    jrr.getDrive().trajectoryBuilder(jrr.getPos())
                            .splineToLinearHeading(new Pose2d(forwardDistance, forwardDistance, 0),0)
                            .build()
            );

            Thread.sleep(1000);

            jrr.getDrive().followTrajectory(
                    jrr.getDrive().trajectoryBuilder(jrr.getPos(), true)
                            .lineToLinearHeading(new Pose2d(0, 0, 0))
                            .build()
            );

            Thread.sleep(1000);
        }
        stopJRRThread.join();
        stop();
    }
}
