package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.blueIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.carouselController;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.interfaceTag;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.primaryControllerMode;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.currentState;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.driveSpeedStrafe;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.primaryGamepad;

import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.secondaryGamepad;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.redIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.roadrunnerHandler;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.slideController;
import static org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp.slideHandler;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp;
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
    private static boolean isPressingLeftTrigger = false, isPressingRightTrigger = false;

    /**
     * Secondary Button Logic
     */
    private static boolean isPressingSecondaryY = false;

    /**
     * Duck Spinner Logic
     */
    public static boolean duckspinnerSpinning = false;

    /**
     * Flags
     */
    public static boolean LEFT_DPAD_COMPARE_FLAG = false;

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

    public static Boolean invertedDuckSpinner = false;
    public static Thread asyncDuck = new Thread(() -> {
        if(invertedDuckSpinner) {
            duckspinnerSpinning = true;
            carouselController.spinOneDuck(false);
            duckspinnerSpinning = false;
            invertedDuckSpinner = false;
        } else {
            duckspinnerSpinning = true;
            carouselController.spinOneDuck();
            duckspinnerSpinning = false;
        }
    });

    public static Boolean runningAsyncSlideExtend = false;
    public static Thread asyncExtendSlide = new Thread(() -> {
        runningAsyncSlideExtend = true;
        if(slideController.slideMotor.getCurrentPosition() < slideController.slideMotorPosition_BUCKET_OUT) {
            slideController.extendSlide();
            driveSpeedStrafe = Configuration.SlowMovementStrafeMultiplier;
        } else if(slideController.slideMotor.getCurrentPosition() > slideController.slideMotorPosition_BUCKET_OUT) {
            slideController.retractSlide();
            if(currentSegmentPosition != SegmentPositions.EXTEND) {
                currentSegmentPosition = SegmentPositions.EXTEND;
            }
            driveSpeedStrafe = 1;
        }
        runningAsyncSlideExtend = false;
    });

    /**
     * Secondary Gamepad
     */
    public static Thread secondaryRuntime = new Thread(() -> {
        while(currentState == TeleOpRobotStates.RUNNING || currentState.equals(TeleOpRobotStates.INITIALIZING)) {
            if(secondaryGamepad.y && !isPressingSecondaryY) {
                if(duckspinnerSpinning == false) {
                    asyncDuck.start();
                }
                isPressingSecondaryY = true;
            } else if(!secondaryGamepad.y && isPressingSecondaryY) {
                isPressingSecondaryY = false;
            }
        }
    });

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

                    /**
                     * B BUTTON
                     * @usage Extend / Retract Slide
                     */
                    if(primaryGamepad.b && !isPressingB) {
                        if(!runningAsyncSlideExtend && !asyncExtendSlide.isAlive()) {
                            asyncExtendSlide.start();
                        }
                        isPressingB = true;
                    } else if(!primaryGamepad.b && isPressingB) {
                        isPressingB = false;
                    }

                    /**
                     * A BUTTON
                     * @usage Deposits Cube and Resets LED Strip To Default Color
                     */
                    if(primaryGamepad.a && !isPressingA && primaryControllerMode == ControllerModes.PRIMARY) {
                        slideController.dropCube();
                        BlueTeleOp.resetColor();
                        isPressingA = true;
                    } else if(!primaryGamepad.a && isPressingA && primaryControllerMode == ControllerModes.PRIMARY) {
                        isPressingA = false;
                    }

                    /**
                     * X BUTTON
                     * @usage Collect Capstone (4 Stage) (PICKUP, EXTEND, DOWN, RETRACT)
                     */
                    if(primaryGamepad.x && !isPressingX) {
                        switch (currentSegmentPosition) {
                            case EXTEND:
                                slideController.collectCapstone();
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
                    /**
                     * Y BUTTON
                     * @usage Duck Spinner
                     */
                    if(primaryGamepad.y && !isPressingY) {
                        if(duckspinnerSpinning == false) {
                            asyncDuck.start();
                        }
                        isPressingY = true;
                    } else if(!primaryGamepad.y && isPressingY) {
                        isPressingY = false;
                    }

                    /**
                     * RIGHT BUMPER
                     * @usage Detect White Line and deposit
                     */
                    if(primaryGamepad.right_bumper && !isPressingRightBumper) {
                        if(duckspinnerSpinning == false) {
                            Log.d(interfaceTag, "Running duck red");
                            invertedDuckSpinner = true;
                            asyncDuck.start();
                        }
                        isPressingRightBumper = true;
                    } else if(!primaryGamepad.right_bumper && isPressingRightBumper) {
                        isPressingRightBumper = false;
                    }

                    /**
                     * LEFT DPAD
                     * @usage Left Intake (Blue Intake)
                     * @info Uses LEFT_DPAD_COMPARE_FLAG to sync the blue intake drop with the program to avoid accidental
                     * thread creation
                     */
                    if(primaryGamepad.dpad_left && !isPressingLeftDpad) {
                        if(!LEFT_DPAD_COMPARE_FLAG) {
                            LEFT_DPAD_COMPARE_FLAG = true;
                            new Thread(() -> {
                                if(blueIntake.getState() == IntakeState.DUMP && slideController.getSlideMotorPosition() < slideController.slideMotorPosition_BUCKET_OUT) {
                                    blueIntake.setState(IntakeState.BASE);
                                } else {
                                    blueIntake.setState(IntakeState.DUMP);
                                }
                                LEFT_DPAD_COMPARE_FLAG = false;
                            }).start();
                        } else {
                            Log.d(interfaceTag, "Tried running a task again");
                        }
                        isPressingLeftDpad = true;
                    } else if(!primaryGamepad.dpad_left && isPressingLeftDpad) {
                        isPressingLeftDpad = false;
                    }

                    /**
                     * RIGHT DPAD
                     * @usage Right Intake (Red Intake)
                     */
                    if(primaryGamepad.dpad_right && !isPressingRightDpad) {
                        if(redIntake.getState() == IntakeState.DUMP && slideController.getSlideMotorPosition() < slideController.slideMotorPosition_BUCKET_OUT) {
                            redIntake.setState(IntakeState.BASE);
                        } else {
                            redIntake.setState(IntakeState.DUMP);
                        }
                        isPressingRightDpad = true;
                    } else if(!primaryGamepad.dpad_right && isPressingRightDpad) {
                        isPressingRightDpad = false;
                    }

                    /**
                     * UP DPAD
                     * @usage Increments Slide Up
                     */
                    if(primaryGamepad.dpad_up) {
                        slideController.incrementVerticalServo(Configuration.DefaultVerticalSlideIncrement);
                    }

                    /**
                     * DOWN DPAD
                     * @usage Increments Slide Down
                     */
                    if(primaryGamepad.dpad_down) {
                        slideController.incrementVerticalServo(-Configuration.DefaultVerticalSlideIncrement);
                    }

                    /**
                     * LEFT TRIGGER
                     * @usage Right Intake (Red Intake)
                     */
                    if(primaryGamepad.left_trigger > Configuration.leftTriggerTreshold && !isPressingLeftTrigger) {
                        if(blueIntake.isDown()) {
                            blueIntake.setState(IntakeState.BASE);
                            redIntake.setState(IntakeState.BASE);
                        } else if(!blueIntake.isDown()) {
                            blueIntake.setState(IntakeState.DUMP);
                            redIntake.setState(IntakeState.DUMP);
                        }
                        isPressingLeftTrigger = true;
                    } else if(primaryGamepad.left_trigger < Configuration.leftTriggerTreshold && isPressingLeftTrigger) {
                        isPressingLeftTrigger = false;
                    }
                    break;
                case SECONDARY:

                    /**
                     * RIGHT STICK (Y VAL)
                     * @usage Manual Slide Controller
                     */
                    if(primaryGamepad.right_stick_y > Configuration.rightStickXLimitTrigger || primaryGamepad.right_stick_y < (Configuration.rightStickXLimitTrigger * -1)) {
                        slideHandler.manualSlideController((int) primaryGamepad.right_stick_y);
                    }

                    /**
                     * A BUTTON
                     * @usage Slowmode Toggle
                     */
                    if(primaryGamepad.a && !isPressingA && primaryControllerMode == ControllerModes.SECONDARY) {
                        if(driveSpeedStrafe == Configuration.SlowMovementStrafeMultiplier) {
                            driveSpeedStrafe = 1;
                        } else if(driveSpeedStrafe == 1) {
                            driveSpeedStrafe = Configuration.SlowMovementStrafeMultiplier;
                        }
                        isPressingA = true;
                    } else if(!primaryGamepad.a && isPressingA && primaryControllerMode == ControllerModes.SECONDARY) {
                        isPressingA = false;
                    }

                    /**
                     * X BUTTON
                     * @usage Reverse + Drop Left Intake
                     */
                    if(primaryGamepad.x && !isPressingX) {
                        if(!blueIntake.isReversed()) {
                            blueIntake.setState(IntakeState.REVERSE);
                        } else if(blueIntake.isReversed()) {
                            blueIntake.setState(IntakeState.BASE);
                        }
                        isPressingX = true;
                    } else if(!primaryGamepad.x && isPressingX) {
                        isPressingX = false;
                    }

                    /**
                     * Y BUTTON
                     * @usage Far extend
                     */
                    if(primaryGamepad.y && !isPressingY) {
                        switch(slideController.targetParams) {
                            case TOP_DEPOSIT:
                                slideController.targetParams = SlideTarget.SHARED_TWO;
                                break;

                            case SHARED_TWO:
                                slideController.targetParams = SlideTarget.FLOOR_SHARED_DEPOSIT;
                                break;

                            case FLOOR_SHARED_DEPOSIT:
                                slideController.targetParams = SlideTarget.TOP_DEPOSIT;
                                break;
                        }
                        isPressingY = true;
                    } else if(!primaryGamepad.y && isPressingY) {
                        isPressingY = false;
                    }

                    /**
                     * LEFT TRIGGER
                     * @usage Interrupt The Slide
                     */
                    if(primaryGamepad.left_trigger > 0.2) {
                        asyncExtendSlide.interrupt();
                        runningAsyncSlideExtend = false;
                    }


                    /**
                     * LEFT DPAD
                     * @usage Left Movement
                     */
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

                    /**
                     * RIGHT DPAD
                     * @usage Right Movement
                     */
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

                    /**
                     * UP DPAD
                     * @usage Forward Movement
                     */
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

                    /**
                     * DOWN DPAD
                     * @usage Backwards Movement
                     */
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
