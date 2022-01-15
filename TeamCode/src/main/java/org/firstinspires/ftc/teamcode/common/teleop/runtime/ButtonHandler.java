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
                    if(gamepad.isDown(GamepadKeys.Button.A)) {
                        slideController.incrementVerticalServo(0.1);
                        Log.d(interfaceTag, "MOVING SERVOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
                    }
                    // X
                    if(gamepad.isDown(GamepadKeys.Button.X)) {
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
                    }
                    // Yp
                    if(gamepad.isDown(GamepadKeys.Button.Y)) {
                        carouselController.spinOneDuck(true);
                        Log.d(interfaceTag, ": Spinning duck");
                    }
                    // rb
                    if(gamepad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
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
                    if(gamepad.getRightY() > Configuration.rightStickXLimitTrigger || gamepad.getRightY() < (Configuration.rightStickXLimitTrigger * -1)) {
                        slideHandler.manualSlideController((int)gamepad.getRightY());
                    }



                    break;
            }
        }

    });
}
