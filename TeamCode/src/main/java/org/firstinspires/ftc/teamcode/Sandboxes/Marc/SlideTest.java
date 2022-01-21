package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.Robot;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideTarget;

public class SlideTest extends OpMode {
    Robot robot;
    SlideController slideCtrl;
    IntakeControllerBlue intakeCtrlB;
    IntakeControllerRed intakeCtrlR;
    public static int MODE = 0;

    boolean qA,qB,qX,qY;
    @Override
    public void init() {
        robot = new Robot(hardwareMap,telemetry, AutonomousMode.BlueFull);
        slideCtrl = robot.getSlideCtrl();
        intakeCtrlB = robot.getIntakeCtrlBlue();
        intakeCtrlB.setState(IntakeState.PARK);
        intakeCtrlR = robot.getIntakeCtrlRed();
        intakeCtrlR.setState(IntakeState.PARK);
        msStuckDetectLoop = Integer.MAX_VALUE;
    }

    @Override
    public void loop() {
        if(MODE == 1) {
            slideCtrl.targetParams = SlideTarget.THREE_CAROUSEL;
            slideCtrl.extendSlide();
            while(MODE != 2) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {

                }
            }
            slideCtrl.dropCube();
            while(MODE != 3) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {

                }
            }
            slideCtrl.retractSlide();
        }

    }
}
