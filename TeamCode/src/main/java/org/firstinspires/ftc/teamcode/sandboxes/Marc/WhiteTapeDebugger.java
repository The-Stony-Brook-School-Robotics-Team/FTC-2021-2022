package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.sbs.bears.robotframework.Robot;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.tensorflow.lite.task.text.qa.QaAnswer;

public class WhiteTapeDebugger extends LinearOpMode {
    RoadRunnerController rrctrl;
    Robot robot;
    SlideController slideCtrl;
    TrajectorySequence seq;
    @Override
    public void runOpMode() throws InterruptedException {
        boolean qA = false;

        robot = new Robot(hardwareMap,telemetry, AutonomousMode.TELEOP);
        rrctrl = robot.getRRctrl();
        slideCtrl = robot.getSlideCtrl();

        rrctrl.setPos(startPos);
        seq = rrctrl.getDrive().trajectorySequenceBuilder(startPos)
                .splineToSplineHeading(wallResetPos,-Math.PI)
                .lineToSplineHeading(whiteLinePos)
                .lineToSplineHeading(dropOff)
                .build();
        waitForStart();

        while(opModeIsActive() && !isStopRequested())
        {
            if(gamepad1.a && !qA)
            {
                qA = true;

                rrctrl.getDrive().followTrajectorySequence(seq);

            }
            else if(!gamepad1.a && qA)
            {
                qA = false;
            }
        }
    }
    public void iFoundWhiteLine()
    {
       TrajectorySequence current = rrctrl.getDrive().trajectorySequenceRunner.currentTrajectorySequence;
       if(current.equals(seq))
       {
           
       }
    }

    Pose2d startPos = new Pose2d(28.5,65.5,0);
    Pose2d wallResetPos = new Pose2d(40,70,0);
    Pose2d whiteLinePos = new Pose2d(28.5,65.5,0);
    Pose2d dropOff = new Pose2d(14,65.5,0);
}


