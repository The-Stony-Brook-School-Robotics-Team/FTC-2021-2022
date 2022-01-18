package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.Robot;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.SlideTarget;

public class SlideTest extends OpMode {
    Robot robot;
    SlideController slideCtrl;

    boolean qA,qB,qX,qY;
    @Override
    public void init() {
        robot = new Robot(hardwareMap,telemetry, AutonomousMode.BlueFull);
        slideCtrl = robot.getSlideCtrl();
        msStuckDetectLoop = Integer.MAX_VALUE;
    }

    @Override
    public void loop() {
        if(!qA && gamepad1.a) {
            qA = true;
            slideCtrl.targetParams = SlideTarget.THREE_CAROUSEL;
            slideCtrl.extendSlide();
            while(gamepad1.x) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {

                }
            }
            slideCtrl.dropCube();
            while(gamepad1.x) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {

                }
            }
            slideCtrl.retractSlide();
        }
        if(qA && !gamepad1.a)
        {
            qA = false;
        }
    }
}
