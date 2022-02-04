package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.blueIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.carouselController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.controllerMode;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.currentState;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.driveSpeed;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.gamepad1;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.gamepad2;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.redIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.roadrunnerHandler;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideHandler;

import android.util.Log;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop;
import org.firstinspires.ftc.teamcode.common.teleop.enums.ControllerModes;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideTarget;


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

    private static boolean isPressingSecondaryY = false;

    /**
     * Segment Enums
     */
    public enum SegmentPositions {
        EXTEND,
        EXTEND_TO_HUB,
        DROP,
        RETRACT
    }
    public static SegmentPositions currentSegmentPosition = SegmentPositions.EXTEND;


    /**
     * Working Button Handler Runtime
     */
    public static Thread runtime = new Thread(() -> {
        while(currentState == TeleOpRobotStates.RUNNING || currentState.equals(TeleOpRobotStates.INITIALIZING)) {

            if(gamepad1.left_bumper) {
                controllerMode = ControllerModes.SECONDARY;
            } else {
                controllerMode = ControllerModes.PRIMARY;
            }

            /**
             * Secondary Gamepad
             */
            if(gamepad2.y && !isPressingSecondaryY) {
                carouselController.spinOneDuck();
                isPressingSecondaryY = true;
            } else if(!gamepad2.y && isPressingSecondaryY) {
                isPressingSecondaryY = false;
            }

            /**
             * Primary Gamepad
             */
            switch(controllerMode) {
                case PRIMARY:

                    // B
                    if(gamepad1.b && !isPressingB) {
                        if(slideController.slideMotor.getCurrentPosition() < slideController.slideMotorPosition_BUCKET_OUT) {
                            slideController.extendSlide();
                            driveSpeed = 0.3;
                        } else if(slideController.slideMotor.getCurrentPosition() > slideController.slideMotorPosition_BUCKET_OUT) {
                            slideController.retractSlide();
                            if(currentSegmentPosition != SegmentPositions.EXTEND) {
                                currentSegmentPosition = SegmentPositions.EXTEND;
                            }
                            driveSpeed = 1;
                        }
                        isPressingB = true;
                    } else if(!gamepad1.b && isPressingB) {
                        isPressingB = false;
                    }

                    // A
                    if(gamepad1.a && !isPressingA && controllerMode == ControllerModes.PRIMARY) {
                        slideController.dropCube();
                        OfficialTeleop.resetColor();
                        isPressingA = true;
                    } else if(!gamepad1.a && isPressingA && controllerMode == ControllerModes.PRIMARY) {
                        isPressingA = false;
                    }

                    // X
                    if(gamepad1.x && !isPressingX) {
                        switch (currentSegmentPosition) {
                            case EXTEND:
                                slideController.collectCapstone();
                                // slideController.incrementVerticalServo(0.1);
                                currentSegmentPosition = SegmentPositions.EXTEND_TO_HUB;
                                break;

                            case EXTEND_TO_HUB:
                                slideController.targetParams = SlideTarget.CAP_FROM_CAROUSEL;
                                slideController.extendSlide();
                                currentSegmentPosition = SegmentPositions.DROP;
                                break;

                            case DROP:
                                slideController.dropCube();
                                currentSegmentPosition = SegmentPositions.RETRACT;
                                break;

                            case RETRACT:
                                slideController.retractSlide();
                                slideController.targetParams = SlideTarget.TOP_CAROUSEL;
                                currentSegmentPosition = SegmentPositions.EXTEND;
                                break;
                        }
                        isPressingX = true;
                    } else if(!gamepad1.x && isPressingX) {
                        isPressingX = false;
                    }

                    // Y
                    if(gamepad1.y && !isPressingY) {
                        carouselController.spinOneDuck();
                        isPressingY = true;
                    } else if(!gamepad1.y && isPressingY) {
                        isPressingY = false;
                    }

                    // rb
                    if(gamepad1.right_bumper && OfficialTeleop.normalizedColorSensor.getNormalizedColors().alpha > Configuration.colorSensorWhiteAlpha) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.WAREHOUSE_AUTO_TURN);
                        }
                    }
                    // Left dpad
                    if(gamepad1.dpad_left && !isPressingLeftDpad) {
                        if(blueIntake.getState() == IntakeState.PARK && slideController.getSlideMotorPosition() < slideController.slideMotorPosition_BUCKET_OUT) {
                            blueIntake.setState(IntakeState.BASE);
                        } else {
                            blueIntake.setState(IntakeState.PARK);
                        }
                        isPressingLeftDpad = true;
                    } else if(!gamepad1.dpad_left && isPressingLeftDpad) {
                        isPressingLeftDpad = false;
                    }

                    // Right dpad
                    if(gamepad1.dpad_right && !isPressingRightDpad) {
                        if(redIntake.getState() == IntakeState.PARK && slideController.getSlideMotorPosition() < slideController.slideMotorPosition_BUCKET_OUT) {
                            redIntake.setState(IntakeState.BASE);
                        } else {
                            redIntake.setState(IntakeState.PARK);
                        }
                        isPressingRightDpad = true;
                    } else if(!gamepad1.dpad_right && isPressingRightDpad) {
                        isPressingRightDpad = false;
                    }

                    // Up dpad
                    if(gamepad1.dpad_up) {
                        slideController.incrementVerticalServo(Configuration.DefaultVerticalSlideIncrement);
                    }

                    // Down dpad
                    if(gamepad1.dpad_down) {
                        slideController.incrementVerticalServo(-Configuration.DefaultVerticalSlideIncrement);
                    }
                    break;
                case SECONDARY:

                    // Manual Extension
                    if(gamepad1.right_stick_y > Configuration.rightStickXLimitTrigger || gamepad1.right_stick_y < (Configuration.rightStickXLimitTrigger * -1)) {
                        slideHandler.manualSlideController((int) gamepad1.right_stick_y);
                    }

                    // A
                    if(gamepad1.a && !isPressingA && controllerMode == ControllerModes.SECONDARY) {
                        if(driveSpeed == 0.3) {
                            driveSpeed = 1;
                        } else if(driveSpeed == 1) {
                            driveSpeed = 0.3;
                        }
                        isPressingA = true;
                    } else if(!gamepad1.a && isPressingA && controllerMode == ControllerModes.SECONDARY) {
                        isPressingA = false;
                    }


                    // Y
                    if(gamepad1.y && !isPressingY && controllerMode == ControllerModes.SECONDARY) {
                        OfficialTeleop.driveSpeed += .01;
                        isPressingY = true;
                    } else if(!gamepad1.y && isPressingY && controllerMode == ControllerModes.SECONDARY) {
                        isPressingY = false;
                    }

                    // B
                    if(gamepad1.b && !isPressingB && controllerMode == ControllerModes.SECONDARY) {
                        OfficialTeleop.driveSpeed -= .01;
                        isPressingB = true;
                    } else if(!gamepad1.b && isPressingB && controllerMode == ControllerModes.SECONDARY) {
                        isPressingB = false;
                    }

                    // X
                    // TODO: FREE BUTTON
                    if(gamepad1.x && !isPressingX) {

                        isPressingX = true;
                    } else if(!gamepad1.x && isPressingX) {
                        isPressingX = false;
                    }
                    // Left Dpad
                    if(gamepad1.dpad_left && !isPressingLeftDpad) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.FORWARD);
                        } else {
                            Log.d(interfaceTag, "Tried scheduling a movement while executor was busy, Movement: " + RoadrunnerHandler.MovementTypes.FORWARD);
                        }
                        isPressingLeftDpad = true;
                    } else if(!gamepad1.dpad_left && isPressingLeftDpad) {
                        isPressingLeftDpad = false;
                    }
                    // Right Dpad
                    if(gamepad1.dpad_right && !isPressingRightDpad) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.BACK);
                        } else {
                            Log.d(interfaceTag, "Tried scheduling a movement while executor was busy, Movement: " + RoadrunnerHandler.MovementTypes.BACK);
                        }
                        isPressingRightDpad = true;
                    } else if(!gamepad1.dpad_right && isPressingRightDpad) {
                        isPressingRightDpad = false;
                    }
                    // Up Dpad
                    if(gamepad1.dpad_up && !isPressingUpDpad) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.RIGHT);
                        } else {
                            Log.d(interfaceTag, "Tried scheduling a movement while executor was busy, Movement: " + RoadrunnerHandler.MovementTypes.RIGHT);
                        }
                        isPressingUpDpad = true;
                    } else if(!gamepad1.dpad_up && isPressingUpDpad) {
                        isPressingUpDpad = false;
                    }
                    // Down Dpad
                    if(gamepad1.dpad_down && !isPressingDownDpad) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.LEFT);
                        } else {
                            Log.d(interfaceTag, "Tried scheduling a movement while executor was busy, Movement: " + RoadrunnerHandler.MovementTypes.LEFT);
                        }
                        isPressingDownDpad = true;
                    } else if(!gamepad1.dpad_down && isPressingDownDpad) {
                        isPressingDownDpad = false;
                    }
                    break;
            }
        }
    });
}
