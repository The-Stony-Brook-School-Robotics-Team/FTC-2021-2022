package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.blueIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.carouselController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.primaryControllerMode;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.currentState;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.driveSpeed;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.primaryGamepad;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.secondaryGamepad;
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
    public static String primaryInterfaceTag = "Primary Button Handler";
    public static String secondaryInterfaceTag = "Secondary Button Handler";

    /**
     * Button Logic
     */
    private static boolean isPressingX = false, isPressingY = false, isPressingB = false, isPressingA = false;
    private static boolean isPressingLeftDpad = false, isPressingRightDpad = false, isPressingUpDpad = false, isPressingDownDpad = false;
    private static boolean isPressingLeftBumper = false, isPressingRightBumper = false;

    /**
     * Secondary Button Logic
     */
    private static boolean isPressingSecondaryY = false;

    /**
     * Duck Spinner Logic
     */
    public static boolean duckspinnerSpinning = false;

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
     * Secondary Gamepad
     */
    public static Thread secondaryRuntime = new Thread(() -> {
        while(currentState == TeleOpRobotStates.RUNNING || currentState.equals(TeleOpRobotStates.INITIALIZING)) {
            if(secondaryGamepad.y && !isPressingSecondaryY) {
                if(duckspinnerSpinning == false) {
                    new Thread(() -> {
                        duckspinnerSpinning = true;
                        carouselController.spinOneDuck();
                        duckspinnerSpinning = false;
                    }).run();
                }
                isPressingSecondaryY = true;
            } else if(!secondaryGamepad.y && isPressingSecondaryY) {
                isPressingSecondaryY = false;
            }
        }
    });

    /**
     * Primary Gamepad
     */
    public static Thread primaryRuntime = new Thread(() -> {
        while(currentState == TeleOpRobotStates.RUNNING || currentState.equals(TeleOpRobotStates.INITIALIZING)) {

            /**
             * Primary Gamepad Shift
             */
            if(primaryGamepad.left_bumper) {
                primaryControllerMode = ControllerModes.SECONDARY;
            } else {
                primaryControllerMode = ControllerModes.PRIMARY;
            }

            /**
             * Primary Gamepad
             */
            switch(primaryControllerMode) {
                case PRIMARY:

                    // B
                    if(primaryGamepad.b && !isPressingB) {
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
                    } else if(!primaryGamepad.b && isPressingB) {
                        isPressingB = false;
                    }

                    // A
                    if(primaryGamepad.a && !isPressingA && primaryControllerMode == ControllerModes.PRIMARY) {
                        slideController.dropCube();
                        OfficialTeleop.resetColor();
                        isPressingA = true;
                    } else if(!primaryGamepad.a && isPressingA && primaryControllerMode == ControllerModes.PRIMARY) {
                        isPressingA = false;
                    }

                    // X
                    if(primaryGamepad.x && !isPressingX) {
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
                    } else if(!primaryGamepad.x && isPressingX) {
                        isPressingX = false;
                    }

                    // Y
                    if(primaryGamepad.y && !isPressingY) {
                        if(duckspinnerSpinning == false) {
                            new Thread(() -> {
                                duckspinnerSpinning = true;
                                carouselController.spinOneDuck();
                                duckspinnerSpinning = false;
                            }).run();
                        }
                        isPressingY = true;
                    } else if(!primaryGamepad.y && isPressingY) {
                        isPressingY = false;
                    }

                    // rb
                    if(primaryGamepad.right_bumper && OfficialTeleop.normalizedColorSensor.getNormalizedColors().alpha > Configuration.colorSensorWhiteAlpha) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.WAREHOUSE_AUTO_TURN);
                        }
                    }
                    // Left dpad
                    if(primaryGamepad.dpad_left && !isPressingLeftDpad) {
                        if(blueIntake.getState() == IntakeState.PARK && slideController.getSlideMotorPosition() < slideController.slideMotorPosition_BUCKET_OUT) {
                            blueIntake.setState(IntakeState.BASE);
                        } else {
                            blueIntake.setState(IntakeState.PARK);
                        }
                        isPressingLeftDpad = true;
                    } else if(!primaryGamepad.dpad_left && isPressingLeftDpad) {
                        isPressingLeftDpad = false;
                    }

                    // Right dpad
                    if(primaryGamepad.dpad_right && !isPressingRightDpad) {
                        if(redIntake.getState() == IntakeState.PARK && slideController.getSlideMotorPosition() < slideController.slideMotorPosition_BUCKET_OUT) {
                            redIntake.setState(IntakeState.BASE);
                        } else {
                            redIntake.setState(IntakeState.PARK);
                        }
                        isPressingRightDpad = true;
                    } else if(!primaryGamepad.dpad_right && isPressingRightDpad) {
                        isPressingRightDpad = false;
                    }

                    // Up dpad
                    if(primaryGamepad.dpad_up) {
                        slideController.incrementVerticalServo(Configuration.DefaultVerticalSlideIncrement);
                    }

                    // Down dpad
                    if(primaryGamepad.dpad_down) {
                        slideController.incrementVerticalServo(-Configuration.DefaultVerticalSlideIncrement);
                    }
                    break;
                case SECONDARY:

                    // Manual Extension
                    if(primaryGamepad.right_stick_y > Configuration.rightStickXLimitTrigger || primaryGamepad.right_stick_y < (Configuration.rightStickXLimitTrigger * -1)) {
                        slideHandler.manualSlideController((int) primaryGamepad.right_stick_y);
                    }

                    // A
                    if(primaryGamepad.a && !isPressingA && primaryControllerMode == ControllerModes.SECONDARY) {
                        if(driveSpeed == 0.3) {
                            driveSpeed = 1;
                        } else if(driveSpeed == 1) {
                            driveSpeed = 0.3;
                        }
                        isPressingA = true;
                    } else if(!primaryGamepad.a && isPressingA && primaryControllerMode == ControllerModes.SECONDARY) {
                        isPressingA = false;
                    }

                    // Y
                    if(primaryGamepad.y && !isPressingY && primaryControllerMode == ControllerModes.SECONDARY) {
                        OfficialTeleop.driveSpeed += .01;
                        isPressingY = true;
                    } else if(!primaryGamepad.y && isPressingY && primaryControllerMode == ControllerModes.SECONDARY) {
                        isPressingY = false;
                    }

                    // B
                    if(primaryGamepad.b && !isPressingB && primaryControllerMode == ControllerModes.SECONDARY) {
                        OfficialTeleop.driveSpeed -= .01;
                        isPressingB = true;
                    } else if(!primaryGamepad.b && isPressingB && primaryControllerMode == ControllerModes.SECONDARY) {
                        isPressingB = false;
                    }

                    // X
                    // TODO: FREE BUTTON
                    if(primaryGamepad.x && !isPressingX) {
                        slideController.dumperServo.setPosition(slideController.dumperPosition_READY);
                        isPressingX = true;
                    } else if(!primaryGamepad.x && isPressingX) {
                        isPressingX = false;
                    }
                    // Left Dpad
                    if(primaryGamepad.dpad_left && !isPressingLeftDpad) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.FORWARD);
                        } else {
                            Log.d(primaryInterfaceTag, "Tried scheduling a movement while executor was busy, Movement: " + RoadrunnerHandler.MovementTypes.FORWARD);
                        }
                        isPressingLeftDpad = true;
                    } else if(!primaryGamepad.dpad_left && isPressingLeftDpad) {
                        isPressingLeftDpad = false;
                    }
                    // Right Dpad
                    if(primaryGamepad.dpad_right && !isPressingRightDpad) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.BACK);
                        } else {
                            Log.d(primaryInterfaceTag, "Tried scheduling a movement while executor was busy, Movement: " + RoadrunnerHandler.MovementTypes.BACK);
                        }
                        isPressingRightDpad = true;
                    } else if(!primaryGamepad.dpad_right && isPressingRightDpad) {
                        isPressingRightDpad = false;
                    }
                    // Up Dpad
                    if(primaryGamepad.dpad_up && !isPressingUpDpad) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.RIGHT);
                        } else {
                            Log.d(primaryInterfaceTag, "Tried scheduling a movement while executor was busy, Movement: " + RoadrunnerHandler.MovementTypes.RIGHT);
                        }
                        isPressingUpDpad = true;
                    } else if(!primaryGamepad.dpad_up && isPressingUpDpad) {
                        isPressingUpDpad = false;
                    }
                    // Down Dpad
                    if(primaryGamepad.dpad_down && !isPressingDownDpad) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.LEFT);
                        } else {
                            Log.d(primaryInterfaceTag, "Tried scheduling a movement while executor was busy, Movement: " + RoadrunnerHandler.MovementTypes.LEFT);
                        }
                        isPressingDownDpad = true;
                    } else if(!primaryGamepad.dpad_down && isPressingDownDpad) {
                        isPressingDownDpad = false;
                    }
                    break;
            }
        }
    });
}
