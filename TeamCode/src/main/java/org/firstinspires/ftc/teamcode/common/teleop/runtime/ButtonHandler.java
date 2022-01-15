package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.blueIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.carouselController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.controllerMode;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.currentState;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.gamepad;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.redIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideHandler;

import android.util.Log;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop;
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
        Log.d(interfaceTag, "Button Handler Running");
        while(currentState == TeleOpRobotStates.RUNNING || currentState.equals(TeleOpRobotStates.INITIALIZING)) {

            if(gamepad.left_bumper) {
                controllerMode = ControllerModes.SECONDARY;
            } else {
                controllerMode = ControllerModes.PRIMARY;
            }

            switch(controllerMode) {
                case PRIMARY:
                    // A
                    if(gamepad.a && !isPressingA) {
                        slideController.incrementVerticalServo(0.1);
                        isPressingA = true;
                    } else if(!gamepad.a && isPressingA) {
                        isPressingA = false;
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
                        Log.d(interfaceTag, ": Toggled Drive Mode");
                        isPressingX = true;
                    } else if(!gamepad.x && isPressingX) {
                        isPressingX = false;
                    }
                    // Yp
                    if(gamepad.y && !isPressingY) {
                        carouselController.spinOneDuck(true);
                        Log.d(interfaceTag, ": Spinning duck");
                        isPressingY = true;
                    } else if(!gamepad.y && isPressingY) {
                        isPressingY = false;
                    }
                    // rb
                    if(gamepad.right_bumper && !isPressingRightBumper) {
                        if(slideHandler.toggleSlide.isAlive()) {
                            slideHandler.toggleSlide.interrupt();
                            if(slideHandler.isSlideExtended()) {
                                slideHandler.fullyRetract.run();
                            } else if (!slideHandler.isSlideExtended()) {
                                slideHandler.fullyExtend.run();
                            }
                        }
                        slideHandler.toggleSlide.run();
                        isPressingRightBumper = true;
                    } else if(!gamepad.right_bumper && isPressingRightBumper) {
                        isPressingRightBumper = false;
                    }


                    // Left dpad TODO: Uncomment
//                    if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
//                        Log.d(interfaceTag, ": CHECKING LOGIC");
//                        if(redIntake.getState() == IntakeState.PARK) {
//                            redIntake.setState(IntakeState.BASE);
//                            Log.d(interfaceTag, ": Set Red Intake to BASE position");
//                        } else {
//                            redIntake.setState(IntakeState.PARK);
//                            Log.d(interfaceTag, ": Set Red Intake to PARK position");
//                        }
//                        Log.d(interfaceTag, ": FINISHED FUNCTION");
//                    }
                    // Right dpad
//                    if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
//                        if(blueIntake.getState() == IntakeState.PARK) {
//                            blueIntake.setState(IntakeState.BASE);
//                            Log.d(interfaceTag, ": Set Blue Intake to BASE position");
//                        } else {
//                            blueIntake.setState(IntakeState.PARK);
//                            Log.d(interfaceTag, ": Set Blue Intake to PARK position");
//                        }
//                    }


                    break;
                case SECONDARY:
                    if(gamepad.right_stick_y > Configuration.rightStickXLimitTrigger || gamepad.right_stick_y < (Configuration.rightStickXLimitTrigger * -1)) {
                        slideHandler.manualSlideController((int)gamepad.right_stick_y);
                    }



                    break;
            }
        }

    });
}
