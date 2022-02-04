package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.SlideTarget;

@TeleOp(name = "A - Slide Controller Debugger")
public class SlideControllerDebugger extends LinearOpMode
{
    boolean pA = false, pUp = false, pDown = false;
    boolean pY = false;
    SlideController slideController;
    SampleMecanumDrive drive;
    IntakeControllerBlue bu;
    IntakeControllerRed red;
    boolean slideOut = false;
    private boolean qX;
    private boolean pB;
    private boolean pRB;

    State state = State.OFF;
    @Override
    public void runOpMode() throws InterruptedException {
        slideController = new SlideController(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(30, 2,DriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);
        bu = new IntakeControllerBlue(hardwareMap,telemetry);
        red = new IntakeControllerRed(hardwareMap,telemetry);

        //bu.setState(IntakeState.PARK);
//        red.setState(IntakeState.PARK);

        slideController.targetParams = SlideTarget.TOP_DEPOSIT;

        waitForStart();

        while(!isStopRequested()) {

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            state.equals(State.DEPOSIT) ? -gamepad1.right_stick_x*0.2 : -gamepad1.right_stick_x
                    )
            );


            if(gamepad1.y && !pY) {
                slideController.retractSlide();
                pY = true;
            } else if(!gamepad1.y && pY) {
                pY = false;
            }


            if(gamepad1.right_bumper && !pRB) {
                slideController.collectCapstone();
                pRB = true;
            } else if(!gamepad1.right_bumper && pRB) {
                pRB = false;
            }

            if(gamepad1.a && !pA) {
                slideController.dropCube();
                pA = true;
            } else if(!gamepad1.a && pA) {
                pA = false;
            }
            if(gamepad1.b && !pB) {
                slideController.extendSlide();
                pB = true;
            } else if(!gamepad1.b && pB) {
                pB = false;
            }

            if(gamepad1.right_stick_y < -0.02 || gamepad1.right_stick_y > -0.02) {
                //drive.setWeightedDrivePower(new Pose2d());
                double stickValue = gamepad1.right_stick_y * -0.2;
                slideController.incrementEncoderPosition((int) (stickValue * Configuration.DefaultSlideTicks));
                //Log.d("SLIDE TESTER", "Current Slide Motor Ticks: " + slideController.getSlideMotorPosition());
            }

            if(gamepad1.dpad_up && !pUp) {
                drive.setWeightedDrivePower(new Pose2d());
                slideController.incrementVerticalServo(0.02);
                //Log.d("SLIDE TESTER", "Current Slide Servo Position: " + slideController.getVerticalServoPosition());
                pUp = true;
            } else if(!gamepad1.dpad_up && pUp) {
                pUp = false;
            }

            if(gamepad1.dpad_down && !pDown) {
                drive.setWeightedDrivePower(new Pose2d());
                slideController.incrementVerticalServo(-0.02);
                //Log.d("SLIDE TESTER", "Current Slide Servo Position: " + slideController.getVerticalServoPosition());
                pDown = true;
            } else if(!gamepad1.dpad_down && pDown) {
                pDown = false;
            }
            if(gamepad1.x && !qX) {
                qX = true;
                switch(state) {
                    case OFF:
                        slideController.collectCapstone();
                        state = State.LIFT;
                        break;
                    case LIFT:
                        slideController.incrementVerticalServo(0.1);
                        state = State.DEPOSIT;
                        break;
                    case DEPOSIT:
                        slideController.targetParams = SlideTarget.CAP_FROM_CAROUSEL;
                        slideController.extendSlide();
                        state = State.DROP;
                        break;
                    case DROP:
                        slideController.dropCube();
                        slideController.retractSlide();
                        state = State.PULLBACK;
                        break;
                    case PULLBACK:
                        state = State.OFF;
                        break;
                }
            } else if(!gamepad1.x && qX) {
                qX = false;
            }

            telemetry.addData("Slide Ext",slideController.slideMotor.getCurrentPosition());
            telemetry.addData("Slide Angle",slideController.verticalServo.getPosition());
            telemetry.update();



        }



    }
}
enum State {
    OFF,
    REACH,
    LIFT,
    DEPOSIT,
    DROP,
    PULLBACK,
}