package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.official.teleop.v1.Configuration;
import org.firstinspires.ftc.teamcode.drive.DriveConstantsMain;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideTarget;

@Config
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
    public DistanceSensor distanceSensor;
    public static SlideTarget target = SlideTarget.TOP_DEPOSIT;

    State state = State.OFF;
    private boolean qDR;
    private boolean qDL;

    @Override
    public void runOpMode() throws InterruptedException {
        Log.d("SlideControllerDebugger","Requesting init");
        slideController = new SlideController(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(30, 2, DriveConstantsMain.TRACK_WIDTH);
        TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstantsMain.MAX_ACCEL);
        bu = new IntakeControllerBlue(hardwareMap, slideController.blueDumperServo, telemetry);
        red = new IntakeControllerRed(hardwareMap, slideController.redDumperServo, telemetry);
        distanceSensor = bu.distanceSensor;

        //bu.setState(IntakeState.PARK);
//        red.setState(IntakeState.PARK);

        slideController.targetParams = target;

        waitForStart();
        new Thread(()->{
            while(!isStopRequested()) {

                telemetry.addData("Slide Ext",slideController.slideMotor.getCurrentPosition());
            telemetry.addData("Slide Angle",slideController.verticalServo.getPosition());
            telemetry.addData("Intake Dist",distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Slide params",slideController.targetParams);
            if(bu.state.equals(IntakeState.BASE))
            {
                Log.d("SlideControllerDebugger","Distance reading: " + bu.distanceSensor.getDistance(DistanceUnit.MM));
                telemetry.addData("Distance: ",bu.distanceSensor.getDistance(DistanceUnit.MM));
                telemetry.addData("DistanceR: ",red.distanceSensor.getDistance(DistanceUnit.MM));
            }
            telemetry.update();
        }}).start();

        while(!isStopRequested()) {

            if(!(gamepad1.left_trigger > 0.2))
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_x,
                            gamepad1.left_stick_y,
                            state.equals(State.DEPOSIT) ? -gamepad1.right_stick_x*0.2 : -gamepad1.right_stick_x
                    )
            );

            if(gamepad1.dpad_right && !qDR)
            {
                red.setState(red.getState() == IntakeState.BASE ? IntakeState.DUMP : IntakeState.BASE);
                qDR = true;
            }
            else if(!gamepad1.dpad_right && qDR)
            {
                qDR = false;
            }
            if(gamepad1.dpad_left && !qDL)
            {
                bu.setState(bu.getState() == IntakeState.BASE ? IntakeState.DUMP : IntakeState.BASE);
                qDL = true;
            }
            else if(!gamepad1.dpad_left && qDL)
            {
                qDL = false;
            }
            if(gamepad1.y && !pY) {
                drive.setWeightedDrivePower(new Pose2d());
                slideController.retractSlide();
                pY = true;
            } else if(!gamepad1.y && pY) {
                pY = false;
            }


            if(gamepad1.right_bumper && !pRB) {
                drive.setWeightedDrivePower(new Pose2d());
                //bu.scooper.setPosition(0.54);
                slideController.extendDropRetract(SlideTarget.TOP_DEPOSIT);
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
                drive.setWeightedDrivePower(new Pose2d());
                slideController.targetParams = SlideTarget.SHARED_THREE;
                slideController.extendSlide();
                pB = true;
            } else if(!gamepad1.b && pB) {
                pB = false;
            }

            if((gamepad1.right_stick_y < -0.02 || gamepad1.right_stick_y > -0.02) && (gamepad1.left_trigger > 0.2)) {
                double stickValue = gamepad1.right_stick_y * -2;
                slideController.incrementEncoderPosition((int) (stickValue * Configuration.DefaultSlideTicks), true);
            }

            if(gamepad1.dpad_up && !pUp) {
                slideController.incrementVerticalServo(0.02);
                pUp = true;
            } else if(!gamepad1.dpad_up && pUp) {
                pUp = false;
            }

            if(gamepad1.dpad_down && !pDown) {
                slideController.incrementVerticalServo(-0.02);
                pDown = true;
            } else if(!gamepad1.dpad_down && pDown) {
                pDown = false;
            }
            if(gamepad1.x && !qX) {
                qX = true;
                drive.setWeightedDrivePower(new Pose2d());
                slideController.targetParams = SlideTarget.TOP_DEPOSIT;
                slideController.extendSlide();

            } else if(!gamepad1.x && qX) {
                qX = false;
            }





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
