package org.firstinspires.ftc.teamcode.common.tentativeAuton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Localizer Test", group = "----William")
public class LocalizerTest extends LinearOpMode {
    AutonomousBrain brain;
    SampleMecanumDrive RRDrive;

    Thread updatePose = new Thread(() -> {
        while (opModeIsActive() && !Thread.interrupted()) {
            try {
                if (RRDrive.isRunningFollowTrajectory)
                    Thread.sleep(20);

                brain.RRctrl.getDrive().updatePoseEstimate();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        brain.RRctrl.stopRobot();
    });

    @Override
    public void runOpMode() throws InterruptedException {
        brain = new AutonomousBrain(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);
        RRDrive = brain.RRctrl.getDrive();

        waitForStart();

        updatePose.start();

        while (opModeIsActive()) {
            telemetry.addData("X",RRDrive.getPoseEstimate().getX());
            telemetry.addData("Y",RRDrive.getPoseEstimate().getY());
            telemetry.addData("H",RRDrive.getPoseEstimate().getHeading());
            telemetry.update();
        }

        updatePose.interrupt();
        brain.majorState.set(AutonomousBrain.MajorAutonomousState.FINISHED);
        brain.minorState.set(AutonomousBrain.MinorAutonomousState.STOPPED);
        requestOpModeStop();
        stop();
    }
}
