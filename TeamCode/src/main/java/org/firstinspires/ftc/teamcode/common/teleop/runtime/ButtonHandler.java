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

import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotserver.internal.webserver.SessionParametersGenerator;
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

    /**
     * Segment Enums
     */
    private enum SegmentPositions {
        EXTEND,
        EXTEND_TO_HUB,
        DROP,
        RETRACT
    }
    private static SegmentPositions currentSegmentPosition = SegmentPositions.EXTEND;


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
                        OfficialTeleop.resetColor();
                        isPressingA = true;
                    } else if(!gamepad.a && isPressingA && controllerMode == ControllerModes.PRIMARY) {
                        isPressingA = false;
                    }
                    // B
                    if(gamepad.b && !isPressingB) {
                        if(slideController.slideMotor.getCurrentPosition() < slideController.slideMotorPosition_BUCKET_OUT) {
                            slideController.extendSlide();
                        } else if(slideController.slideMotor.getCurrentPosition() > slideController.slideMotorPosition_BUCKET_OUT) {
                            slideController.retractSlide();
                        }
                        isPressingB = true;
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
                    if(gamepad.right_bumper && OfficialTeleop.normalizedColorSensor.getNormalizedColors().alpha > Configuration.colorSensorWhiteAlpha) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.WAREHOUSE_AUTO_TURN);
                        }
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
                        if(!slideHandler.slideMoving) {
                            slideHandler.DuckToTop();
                        }
                        isPressingA = true;
                    } else if(!gamepad.a && isPressingA && controllerMode == ControllerModes.SECONDARY) {
                        isPressingA = false;
                    }
                    // Y
                    // TODO: Move [PRIMARY] [X] to [PRIMARY] [Y], to put [SECONDARY] [X] on [PRIMARY] [X]
                    // X
                    if(gamepad.x && !isPressingX) {
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
                    } else if(!gamepad.x && isPressingX) {
                        isPressingX = false;
                    }
                    // Left Dpad
                    if(gamepad.dpad_left && !isPressingLeftDpad) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.FORWARD);
                        } else {
                            Log.d(interfaceTag, "Tried scheduling a movement while executor was busy, Movement: " + RoadrunnerHandler.MovementTypes.FORWARD);
                        }
                        isPressingLeftDpad = true;
                    } else if(!gamepad.dpad_left && isPressingLeftDpad) {
                        isPressingLeftDpad = false;
                    }
                    // Right Dpad
                    if(gamepad.dpad_right && !isPressingRightDpad) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.BACK);
                        } else {
                            Log.d(interfaceTag, "Tried scheduling a movement while executor was busy, Movement: " + RoadrunnerHandler.MovementTypes.BACK);
                        }
                        isPressingRightDpad = true;
                    } else if(!gamepad.dpad_right && isPressingRightDpad) {
                        isPressingRightDpad = false;
                    }
                    // Up Dpad
                    if(gamepad.dpad_up && !isPressingUpDpad) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.RIGHT);
                        } else {
                            Log.d(interfaceTag, "Tried scheduling a movement while executor was busy, Movement: " + RoadrunnerHandler.MovementTypes.RIGHT);
                        }
                        isPressingUpDpad = true;
                    } else if(!gamepad.dpad_up && isPressingUpDpad) {
                        isPressingUpDpad = false;
                    }
                    // Down Dpad
                    if(gamepad.dpad_down && !isPressingDownDpad) {
                        if(!roadrunnerHandler.isBusy) {
                            roadrunnerHandler.scheduleMovement(RoadrunnerHandler.MovementTypes.LEFT);
                        } else {
                            Log.d(interfaceTag, "Tried scheduling a movement while executor was busy, Movement: " + RoadrunnerHandler.MovementTypes.LEFT);
                        }
                        isPressingDownDpad = true;
                    } else if(!gamepad.dpad_down && isPressingDownDpad) {
                        isPressingDownDpad = false;
                    }
                    break;
            }
            Log.d(interfaceTag, "Interface still running");
        }
    });
}
