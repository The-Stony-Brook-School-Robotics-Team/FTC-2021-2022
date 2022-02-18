package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.blueIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.carouselController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.interfaceTag;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.primaryControllerMode;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.currentState;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.driveSpeedStrafe;
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

import java.util.concurrent.atomic.AtomicReference;


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

    public static Thread asyncDuck = new Thread(() -> {
        duckspinnerSpinning = true;
        carouselController.spinOneDuck();
        duckspinnerSpinning = false;
    });

    public static AtomicReference<Boolean> runningAsyncSlideExtend = new AtomicReference<>();
    public static Thread asyncExtendSlide = new Thread(() -> {
        // Set the atomic boolean to true (slide function is not finished / in progress)
        runningAsyncSlideExtend.set(true);
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
        // Set the atomic boolean to false (slide function is finished)
        runningAsyncSlideExtend.set(false);
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
                        if(!runningAsyncSlideExtend.get() && !asyncExtendSlide.isAlive()) {
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
                        OfficialTeleop.resetColor();
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
                    //TODO i changed this sorry
                    if(primaryGamepad.right_bumper) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.WAREHOUSE_AUTO_TURN);
                        }
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
                        } else if(!blueIntake.isDown()) {
                            blueIntake.setState(IntakeState.DUMP);
                        }
                        isPressingLeftTrigger = true;
                    } else if(primaryGamepad.left_trigger < Configuration.leftTriggerTreshold && isPressingLeftTrigger) {
                        isPressingLeftTrigger= false;
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
                     * @usage Slide Extension Interrupt
                     */
                    if(primaryGamepad.y && !isPressingY) {
                        asyncExtendSlide.interrupt();
                        runningAsyncSlideExtend.set(false);
                        isPressingY = true;
                    } else if(!primaryGamepad.y && isPressingY) {
                        isPressingY = false;
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
