package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.blueIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.carouselController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.controllerMode;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.currentState;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.gamepad;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.movementHandler;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.redIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.roadrunnerHandler;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideHandler;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.common.teleop.enums.ControllerModes;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;
import org.sbs.bears.robotframework.enums.IntakeState;



public class ButtonHandler {

    /**
     * Interface Tag
     */
    public static String interfaceTag = "Button Handler";

    /**
     * Button Logic
     */
    private static boolean isPressingX = false, isPressingY = false, isPressingB = false, isPressingA = false;
    private static boolean isPressingLeftDpad = false, isPressingRightDpad = false, isPressingUpDpad = false, isPressingDownDpad = false;
    private static boolean isPressingLeftBumper = false, isPressingRightBumper = false;

    /**
     * Working Button Handler Runtime
     */
    public static Thread runtime = new Thread(() -> {
        while(currentState == TeleOpRobotStates.RUNNING || currentState.equals(TeleOpRobotStates.INITIALIZING)) {

            if(movementHandler.currentDriverMode != MovementHandler.DriverMode.DRIVER) {
                RoadrunnerHandler.
            }

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
                        roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.TURN_ABOUT_WHEEL);
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
                        carouselController.spinOneDuck();
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
