package org.firstinspires.ftc.teamcode.sandboxes.William;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.sbs.bears.robotframework.controllers.JRR;

@TeleOp(name = "JRRTest", group = "JRR")
public class JRRTest extends LinearOpMode {
    JRR jrr = new JRR(hardwareMap, gamepad1);

    @Override
    public void runOpMode() throws InterruptedException {

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

        jrr.setCurrentPosition(new Pose2d());
        while (opModeIsActive()) {
            jrr.getDrive().followTrajectory(
                    jrr.getDrive().trajectoryBuilder(jrr.getCurrentPosition())
                            .lineToLinearHeading(new Pose2d(40, 0, 0))
                            .build()
            );

            Thread.sleep(1000);

            jrr.getDrive().followTrajectory(
                    jrr.getDrive().trajectoryBuilder(jrr.getCurrentPosition(), true)
                            .lineToLinearHeading(new Pose2d(0, 0, 0))
                            .build()
            );
        }
        stopJRRThread.join();
        stop();
    }
}
