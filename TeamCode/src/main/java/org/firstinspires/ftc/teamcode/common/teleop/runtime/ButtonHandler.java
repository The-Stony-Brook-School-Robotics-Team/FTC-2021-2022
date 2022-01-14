package org.firstinspires.ftc.teamcode.common.teleop.runtime;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.blueIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.carouselController;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.controllerMode;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.currentState;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.gamepad;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.redIntake;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slideHandler;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.slowmodeToggled;

import android.util.Log;

import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop;
import org.firstinspires.ftc.teamcode.common.teleop.enums.ControllerModes;
import org.firstinspires.ftc.teamcode.common.teleop.enums.TeleOpRobotStates;
import org.sbs.bears.robotframework.enums.IntakeState;



public class ButtonHandler {

    private static String interfaceTag = "ButtonHandler";

    /**
     * Working Button Handler Runtime
     */
    public Thread runtime = new Thread(() -> {
        while(currentState == TeleOpRobotStates.RUNNING) {
            if(gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                controllerMode = ControllerModes.SECONDARY;
            } else {
                controllerMode = ControllerModes.PRIMARY;
            }

            switch(controllerMode) {
                case PRIMARY:
                    // A
                    if(gamepad.wasJustPressed(GamepadKeys.Button.A)) {
                        // TODO: Add open claw function
                    } else {
                        // TODO: Add ensure claw is closed function
                    }
                    // X
                    if(gamepad.wasJustPressed(GamepadKeys.Button.X)) {
                        slowmodeToggled = !slowmodeToggled;
                        Log.d(interfaceTag, "Slowmode toggled");
                    }
                    // Yp
                    if(gamepad.isDown(GamepadKeys.Button.Y)) {
                        carouselController.spinOneDuck(true);
                        Log.d(interfaceTag, "Spinning duck");
                    }
                    // rb
                    if(gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        if(slideHandler.toggleSlide.isAlive()) {
                            slideHandler.toggleSlide.interrupt();
                            if(slideHandler.isSlideExtended()) {
                                slideHandler.fullyRetract.run();
                            } else if (!slideHandler.isSlideExtended()) {
                                slideHandler.fullyExtend.run();
                            }
                        }
                        slideHandler.toggleSlide.run();
                    }
                    // Left dpad
                    if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                        Log.d(interfaceTag, "CHECKING LOGIC");
                        if(redIntake.getState() == IntakeState.PARK) {
                            redIntake.setState(IntakeState.BASE);
                            Log.d(interfaceTag, "Set Red Intake to BASE position");
                        } else {
                            redIntake.setState(IntakeState.PARK);
                            Log.d(interfaceTag, "Set Red Intake to PARK position");
                        }
                        Log.d(interfaceTag, "FINISHED FUNCTION");
                    }
                    // Right dpad
                    if(gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                        if(blueIntake.getState() == IntakeState.PARK) {
                            blueIntake.setState(IntakeState.BASE);
                            Log.d(interfaceTag, "Set Blue Intake to BASE position");
                        } else {
                            blueIntake.setState(IntakeState.PARK);
                            Log.d(interfaceTag, "Set Blue Intake to PARK position");
                        }
                    }


                    break;
                case SECONDARY:
                    Log.d("SHEEEEEEEEEEEEEEEEEEEEEEEEEESH", String.valueOf(gamepad.getRightY()));
                    if(gamepad.getRightY() > Configuration.rightStickXLimitTrigger) {
                        slideHandler.manualSlideController((int)gamepad.getRightY());
                    }



                    break;
            }
        }

    });
}
