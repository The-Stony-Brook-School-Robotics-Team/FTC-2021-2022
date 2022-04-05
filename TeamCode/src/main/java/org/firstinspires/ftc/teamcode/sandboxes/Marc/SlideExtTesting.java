package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.common.tentativeAuton.AutonomousBrain;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.AutomaticSlide;
import org.jetbrains.annotations.TestOnly;
import org.sbs.bears.robotframework.Robot;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeState;
@TeleOp
public class SlideExtTesting extends LinearOpMode {
    private boolean pUp;
    private boolean pDown;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap,telemetry, AutonomousMode.TELEOP);
        SlideController slideCtrl = robot.getSlideCtrl();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(AutonomousBrain.startPositionBlue);
        new Thread(()->{
            while(opModeIsActive() && !isStopRequested()) {
                telemetry.addData("Slide Ext",slideCtrl.slideMotor.getCurrentPosition());
                telemetry.addData("Slide Angle",slideCtrl.verticalServo.getPosition());
                telemetry.addData("Slide params",slideCtrl.targetParams);
                //telemetry.addData("Robot Distance",rrCtrl.distanceTo(AutomaticSlide.blueShippingHub));
                telemetry.update();
            }
        }).start();
        
        waitForStart();
        boolean qA = false;
        boolean qB = false;
        while(opModeIsActive() && !isStopRequested()) {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
            if (gamepad1.a && !qA)
            {
                qA= true;
                drive.turn(AutomaticSlide.calculateTurnNeeded(drive.getPoseEstimate()));
                //slideCtrl.extendSlideToTicks(AutomaticSlide.calculateSlidePosition(drive.getPoseEstimate()));
            }
            else if(!gamepad1.a && qA)
            {
                qA = false;
            }
            if (gamepad1.b && !qB)
            {
                qB= true;
                slideCtrl.retractSlide();
            }
            else if(!gamepad1.b && qB)
            {
                qB = false;
            }
            if(gamepad1.dpad_up && !pUp) {
                drive.setWeightedDrivePower(new Pose2d());
                slideCtrl.incrementVerticalServo(0.02);
                //Log.d("SLIDE TESTER", "Current Slide Servo Position: " + slideController.getVerticalServoPosition());
                pUp = true;
            } else if(!gamepad1.dpad_up && pUp) {
                pUp = false;
            }

            if(gamepad1.dpad_down && !pDown) {
                drive.setWeightedDrivePower(new Pose2d());
                slideCtrl.incrementVerticalServo(-0.02);
                //Log.d("SLIDE TESTER", "Current Slide Servo Position: " + slideController.getVerticalServoPosition());
                pDown = true;
            } else if(!gamepad1.dpad_down && pDown) {
                pDown = false;
            }
        }
    }
}
