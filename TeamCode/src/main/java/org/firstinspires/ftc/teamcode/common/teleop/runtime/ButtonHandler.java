package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.blueIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.carouselController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.controllerMode;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.currentState;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.drive;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.gamepad;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.redIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideHandler;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.common.teleop.enums.ControllerModes;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.enums.IntakeState;



public class ButtonHandler {

    /**
     * Interface Tag
     */
    public static String interfaceTag = "Button Handler";

    /**
     * Button Logic
     */

    static final TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(30, 2, DriveConstants.TRACK_WIDTH);
    static final TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

    private static boolean isPressingX = false, isPressingY = false, isPressingB = false, isPressingA = false;
    private static boolean isPressingLeftDpad = false, isPressingRightDpad = false, isPressingUpDpad = false, isPressingDownDpad = false;
    private static boolean isPressingLeftBumper = false, isPressingRightBumper = false;

    private static boolean isSlideMoving = false;

    /**
     * Working Button Handler Runtime
     */
    public static Thread runtime = new Thread(() -> {
        while(currentState == TeleOpRobotStates.RUNNING || currentState.equals(TeleOpRobotStates.INITIALIZING)) {

            if(gamepad.left_bumper) {
                controllerMode = ControllerModes.SECONDARY;
            } else {
                controllerMode = ControllerModes.PRIMARY;
            }

            switch(controllerMode) {
                case PRIMARY:
                    // A
                    if(gamepad.a && !isPressingA && controllerMode == ControllerModes.PRIMARY) {
                        slideController.dropCube();
                        isPressingA = true;
                    } else if(!gamepad.a && isPressingA && controllerMode == ControllerModes.PRIMARY) {
                        isPressingA = false;
                    }
                    // B
                    if(gamepad.b && isPressingB) {
                        isPressingB = true;
                        // ADDED BY MARC ON THURSDAY JANUARY 20th 2022 AT 14h13 AT REQUEST OF TIGER AND MR WINSTON
                        // turn 51 deg about RB wheel
                        Log.d("ButtonHandler","Initiating pivot about RB wheel");
                        double angle= Math.toRadians(51);
                        double sideOfRobot = 6;
                        drive.setPoseEstimate(new Pose2d(14,65.5,0));
                        //drive.setWeightedDrivePower(new Pose2d()); // stop robot
                        Pose2d currentPos = drive.getPoseEstimate();
                        //Pose2d targetPos = new Pose2d(currentPos.getX()-(sideOfRobot*Math.cos(angle)+sideOfRobot*Math.sin(angle)),currentPos.getY()-(sideOfRobot*Math.sin(angle)-sideOfRobot*Math.cos(angle)),currentPos.getHeading() - angle);
                        Pose2d target = new Pose2d(5.58,64.47,-Math.toRadians(52));
                        drive.followTrajectory(drive.trajectoryBuilder(currentPos)
                                .lineToSplineHeading(target,velocityConstraint,accelerationConstraint)
                                .build());
                        Log.d("ButtonHandler","End of pivot about RB wheel");
                        // END OF ADDED BY MARC ON THURSDAY JANUARY 20th 2022 AT 14h13 AT REQUEST OF TIGER AND MR WINSTON
                    } else if(!gamepad.b && isPressingB) {
                        isPressingB = false;
                    }
                    // X
                    if(gamepad.x && !isPressingX) {
                        MovementHandler.RunType runType = MovementHandler.getRunType();
                        switch (runType) {
                            case DEFAULT:
                                MovementHandler.setRunType(MovementHandler.RunType.SLOW);
                                break;
                            case SLOW:
                                MovementHandler.setRunType(MovementHandler.RunType.DEFAULT);
                                break;
                        }
                        isPressingX = true;
                    } else if(!gamepad.x && isPressingX) {
                        isPressingX = false;
                    }
                    // Y
                    if(gamepad.y && !isPressingY) {
                        carouselController.spinOneDuck(true);
                        isPressingY = true;
                    } else if(!gamepad.y && isPressingY) {
                        isPressingY = false;
                    }
                    // rb
                    if(gamepad.right_bumper && !isPressingRightBumper) {
                        if(slideController.slideMotor.getCurrentPosition() < slideController.slideMotorPosition_BUCKET_OUT) {
                            slideController.extendSlide();
                        } else if(slideController.slideMotor.getCurrentPosition() > slideController.slideMotorPosition_BUCKET_OUT) {
                            slideController.retractSlide();
                        }
                        isPressingRightBumper = true;
                    } else if(!gamepad.right_bumper && isPressingRightBumper) {
                        isPressingRightBumper = false;
                    }
                    // Left dpad
                    if(gamepad.dpad_left && !isPressingLeftDpad) {
                        if(blueIntake.getState() == IntakeState.PARK && slideController.getSlideMotorPosition() < slideController.slideMotorPosition_BUCKET_OUT) {
                            blueIntake.setState(IntakeState.BASE);
                        } else {
                            blueIntake.setState(IntakeState.PARK);
                        }
                        isPressingLeftDpad = true;
                    } else if(!gamepad.dpad_left && isPressingLeftDpad) {
                        isPressingLeftDpad = false;
                    }
                    redIntake.checkIntake();
                    // Right dpad
                    if(gamepad.dpad_right && !isPressingRightDpad) {
                        if(redIntake.getState() == IntakeState.PARK && slideController.getSlideMotorPosition() < slideController.slideMotorPosition_BUCKET_OUT) {
                            redIntake.setState(IntakeState.BASE);
                        } else {
                            redIntake.setState(IntakeState.PARK);
                        }
                        isPressingRightDpad = true;
                    } else if(!gamepad.dpad_right && isPressingRightDpad) {
                        isPressingRightDpad = false;
                    }
                    // Up dpad
                    if(gamepad.dpad_up) {
                        slideController.incrementVerticalServo(Configuration.DefaultVerticalSlideIncrement);
                    }
                    // Down dpad
                    if(gamepad.dpad_down) {
                        slideController.incrementVerticalServo(Configuration.DefaultVerticalSlideIncrement * -1);
                    }
                    break;
                case SECONDARY:
                    // Manual Extension
                    if(gamepad.right_stick_y > Configuration.rightStickXLimitTrigger || gamepad.right_stick_y < (Configuration.rightStickXLimitTrigger * -1)) {
                        slideHandler.manualSlideController((int)gamepad.right_stick_y);
                    }
                    // A
                    if(gamepad.a && !isPressingA && controllerMode == ControllerModes.SECONDARY) {
                        if(slideHandler.slideMoving == false) {
                            slideHandler.DuckToMiddle();
                        }
                        isPressingA = true;
                    } else if(!gamepad.a && isPressingA && controllerMode == ControllerModes.SECONDARY) {
                        isPressingA = false;
                    }
                    break;
            }
        }

    });
}
