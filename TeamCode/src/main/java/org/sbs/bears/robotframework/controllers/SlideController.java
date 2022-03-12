
package org.sbs.bears.robotframework.controllers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.common.teleop.BlueTeleOp;
import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.enums.SlideState;
import org.sbs.bears.robotframework.enums.SlideTarget;

import java.sql.Time;

@Config
public class SlideController {
    public Servo verticalServo;
    private Servo horizontalServo;
    public Servo blueDumperServo;
    public Servo redDumperServo;
    public DigitalChannel magswitch;


    public DcMotorEx slideMotor;

    public SlideState slideState = SlideState.PARKED;
    public SlideTarget targetParams = SlideTarget.NA;
    private boolean flagToLeave = false;
    private double changePositionSlope;
    private boolean isTeleop;


    public static double SERVO_VELOCITY_CONSTANT = 0.8;
    public static boolean SERVO_TEST = false;

    public static int ASlideExtendPositionOffSet = -700;
    public static int ASlideRetractPositionOffset = 400;

    public SlideController(HardwareMap hardwareMap, Telemetry telemetry) {
        /** Initialization **/
        magswitch = hardwareMap.get(DigitalChannel.class, "stop");
        magswitch.setMode(DigitalChannel.Mode.INPUT);
        verticalServo = hardwareMap.get(Servo.class, "vt");
        // horizontalServo = hardwareMap.get(Servo.class, "hz");
        blueDumperServo = hardwareMap.get(Servo.class, "du");
        redDumperServo = hardwareMap.get(Servo.class, "rdu");
        slideMotor = hardwareMap.get(DcMotorEx.class, "spool");


        slideMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(0); // should be where it reset to
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        verticalServo.setPosition(vertServoPosition_PARKED);


        blueDumperServo.setPosition(dumperPosition_READY);
        redDumperServo.setPosition(dumperPosition_READY);
        Log.d("SlideController", "Set the dumper servo to ready");
    }

    /**
     * Increments the slide's height to the desired position.
     **/
    private void setHeightToParams(double targetPos) {

        //Only alter the slide's height if not parked
        //TODO if(slideState != PARKED)?
        if (slideState == SlideState.OUT_FULLY || slideState == SlideState.RETRACTING || slideState == SlideState.EXTENDING || slideState == SlideState.TELEOP) {
            double currentPos = verticalServo.getPosition();
            //If the target position is higher than our current, it means we are going up, and vice versa.
            boolean qNeedsToGoUp = (currentPos < targetPos);

            if (qNeedsToGoUp) {
                if (SERVO_TEST) {
                    double beginningServoTime = NanoClock.system().seconds();
                    double beginningServoPosition = verticalServo.getPosition();
                    double currentServoPosition = beginningServoPosition;
                    while (currentServoPosition <= 0.8) {
                        verticalServo.setPosition(currentServoPosition);
                        currentServoPosition = beginningServoPosition + (NanoClock.system().seconds() - beginningServoTime) * SERVO_VELOCITY_CONSTANT;
                    }
                    verticalServo.setPosition(0.8);
                } else {
                    //TODO: Warning STUPID S STUFFS BELOW
                    //Set the servo to a slightly higher position until it reaches its target
                    for (double i = verticalServo.getPosition(); i < targetPos; i += incrementDeltaExtend) {
                        verticalServo.setPosition(Range.clip(i, 0, targetPos));

                        //    try {
                        //         Thread.sleep(1);
                        //     } catch (InterruptedException e) {
                        //         e.printStackTrace();
                        //      }
                        Log.d("SlideController", "Lifted VerticalServo position to " + verticalServo.getPosition());


                    }
                    //double iniTime
                    verticalServo.setPosition(targetPos); // it works don't ask don't tell
                }
            } else { //We are going down
                if (SERVO_TEST) {
                    double beginningServoTime = NanoClock.system().seconds();
                    double beginningServoPosition = verticalServo.getPosition();
                    double currentServoPosition = beginningServoPosition;
                    while (currentServoPosition >= 0.1) {
                        verticalServo.setPosition(currentServoPosition);
                        currentServoPosition = beginningServoPosition - (NanoClock.system().seconds() - beginningServoTime) * SERVO_VELOCITY_CONSTANT;
                    }
                    verticalServo.setPosition(0.8);
                } else {
                    for (double i = verticalServo.getPosition(); i > targetPos; i -= incrementDeltaRetract) {
                        verticalServo.setPosition(Range.clip(i, targetPos, 1));
                        Log.d("SlideController", "Lowered VerticalServo position to " + verticalServo.getPosition());
                        //Sleep to slow things down a bit
                        //TODO can be removed with a decrease of incrementDeltaRetract?
                        //    try {
                        //        Thread.sleep(1);
                        //     } catch (InterruptedException e) {
                        //         e.printStackTrace();
                        //  }
                    }
                }
            }
        } else {
            Log.d("SlideController", "Assert for slide box outside of robot failed; dont lift the slide while the box is inside the robot!");
        }
    }

    /**
     * @param targetPos
     * @param servoVelocity 0.8 is one second of lifting. Using a velocity below 0.8 for capping.
     */
    private void setHeightToParams(double targetPos, double servoVelocity) {
        //Only alter the slide's height if not parked
        //TODO if(slideState != PARKED)?
        if (slideState == SlideState.OUT_FULLY || slideState == SlideState.RETRACTING || slideState == SlideState.EXTENDING || slideState == SlideState.TELEOP) {
            double currentPos = verticalServo.getPosition();
            //If the target position is higher than our current, it means we are going up, and vice versa.
            boolean qNeedsToGoUp = (currentPos < targetPos);

            if (qNeedsToGoUp) {
                if (SERVO_TEST) {
                    double beginningServoTime = NanoClock.system().seconds();
                    double beginningServoPosition = verticalServo.getPosition();
                    double currentServoPosition = beginningServoPosition;
                    while (currentServoPosition <= 0.8) {
                        verticalServo.setPosition(currentServoPosition);
                        currentServoPosition = beginningServoPosition + (NanoClock.system().seconds() - beginningServoTime) * servoVelocity;
                    }
                    verticalServo.setPosition(0.8);
                } else {
                    //TODO: Warning STUPID S STUFFS BELOW
                    //Set the servo to a slightly higher position until it reaches its target
                    for (double i = verticalServo.getPosition(); i < targetPos; i += incrementDeltaExtend) {
                        verticalServo.setPosition(Range.clip(i, 0, targetPos));

                        //    try {
                        //         Thread.sleep(1);
                        //     } catch (InterruptedException e) {
                        //         e.printStackTrace();
                        //      }
                        Log.d("SlideController", "Lifted VerticalServo position to " + verticalServo.getPosition());
                        Log.d("SlideController", "Lifted VerticalServo position to " + Range.clip(i, 0, targetPos));


                    }
                    //double iniTime
                    verticalServo.setPosition(targetPos); // it works don't ask don't tell
                }
            } else { //We are going down
                if (SERVO_TEST) {
                    double beginningServoTime = NanoClock.system().seconds();
                    double beginningServoPosition = verticalServo.getPosition();
                    double currentServoPosition = beginningServoPosition;
                    while (currentServoPosition >= 0.1) {
                        verticalServo.setPosition(currentServoPosition);
                        currentServoPosition = beginningServoPosition - (NanoClock.system().seconds() - beginningServoTime) * servoVelocity;
                    }
                    verticalServo.setPosition(0.8);
                } else {
                    for (double i = verticalServo.getPosition(); i > targetPos; i -= incrementDeltaRetract) {
                        verticalServo.setPosition(Range.clip(i, targetPos, 1));
                        //Sleep to slow things down a bit
                        //TODO can be removed with a decrease of incrementDeltaRetract?
                        //    try {
                        //        Thread.sleep(1);
                        //     } catch (InterruptedException e) {
                        //         e.printStackTrace();
                        //  }
                    }
                }
            }
        } else {
            Log.d("SlideController", "Assert for slide box outside of robot failed; dont lift the slide while the box is inside the robot!");
        }
    }

    /**
     * Extracts the slide, drops, then retracts.
     *
     * @param target the target which represents the height and distance the slide should extend to.
     */
    public void extendDropRetract(SlideTarget target) {
        this.targetParams = target;
        extendSlide();
        if (flagToLeave) {
            return;
        }

        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if (flagToLeave) {
            return;
        }

        dropCube();
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        retractSlide();
        this.targetParams = SlideTarget.NA;
    }

    public void extendDropRetractAuton(SlideTarget target) {
        this.targetParams = target;
        extendSlide();
        if (flagToLeave) {
            return;
        }

        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if (flagToLeave) {
            return;
        }

        dropCube();
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        new Thread(() -> {
            retractSlide();
        }).start();
        Sleep.sleep(300);
        this.targetParams = SlideTarget.NA;
    }

    public void resetSlideMotorConfigs(){
        slideMotor.resetDeviceConfigurationForOpMode();
        slideMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(0); // should be where it reset to
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void extendDropRetract_NewAutonomous(SlideTarget target) {
        this.targetParams = target;
        resetSlideMotorConfigs();

        extendSlide_NewAutonomous();
        dropCube();
        retractSlide_NewAutonomous();

        this.targetParams = SlideTarget.NA;
    }


    public void collectCapstone() {
        // extend a certain amount, then lift, then chill.
        //setHeightToParams(vertServoPosition_GRAB_CAP);
        verticalServo.setPosition(vertServoPosition_GRAB_CAP);
        slideMotor.setTargetPosition(slideMotorPosition_CAP_ON_GROUND);
        slideMotor.setPower(slideMotorPowerGrabCap);
        double TIMER_KILL = 2;
        double currentTime = NanoClock.system().seconds();
        double timeToKill = currentTime + TIMER_KILL;
        while (slideMotor.getCurrentPosition() < slideMotorPosition_CAP_ON_GROUND) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if(timeToKill <= NanoClock.system().seconds())
            {
                hardStopReset();
            }
        }
        slideMotor.setPower(0);
    }

    /**
     * Moves the bucket servo to drop the cube, then returns the servo to original position.
     **/
    public void dropCube() {
        //Checks to make sure the bucket is outside of the slide before dumping
        if (slideMotor.getCurrentPosition() > slideMotorPosition_BUCKET_OUT) {
            if (targetParams == SlideTarget.CAP_FROM_CAROUSEL) {
                incrementDeltaRetract = incrementDeltaRetractCaptsone;
                setHeightToParams(vertServoPosition_CAP_CAROUSEL);
                incrementDeltaRetract = incrementDeltaRetractTeleop;
                slideMotor.setTargetPosition(slideMotorPosition_CAP_FROM_CAROUSEL_RET);
                slideMotor.setPower(slideMotorPowerMovingBack);
                while (slideMotor.getCurrentPosition() < slideMotorPosition_CAP_FROM_CAROUSEL_RET) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                slideMotor.setPower(0);
            } else {
                Log.d("SlideController", "Bucket Eject");
                blueDumperServo.setPosition(dumperPosition_EJECT);
                redDumperServo.setPosition(dumperPosition_EJECT);
                try {
                    Thread.sleep(400); // 250
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            Log.d("SlideController", "Bucket in retract position");
            blueDumperServo.setPosition(dumperPosition_RETRACTING);
            redDumperServo.setPosition(dumperPosition_RETRACTING);
        }
    }

    /**
     * Sets the slide state to retract, does the action for this state (retracts), then sets the state to park.
     **/
    public void retractSlide() {
        slideState = SlideState.RETRACTING;
        doStateAction();
        Log.d("SlideController", "Should be inside " + slideMotor.getCurrentPosition() + " " + verticalServo.getPosition());
        slideState = SlideState.PARKED;
    }

    /**
     * Sets the slide state to extend, does the action for this state (extends), then sets the state to out fully.
     **/
    public void extendSlide() {
        Log.d("SlideController", "bucket should be out");
        slideState = SlideState.EXTENDING;
        doStateAction();
        Log.d("SlideController", "slide should be extended");
        slideState = SlideState.OUT_FULLY;
    }

    /**
     * Sets the slide state to extend, does the action for this state (extends), then sets the state to out fully.
     **/
    public void extendSlide_NewAutonomous() {
        Log.d("SlideController", "bucket should be out");
        slideState = SlideState.EXTENDING;

        int targetPosFinal = 0;
        double verticalServoTargetPos = 0;

        BlueTeleOp.driveSpeedStrafe = Configuration.SlowMovementStrafeMultiplier;
        switch (targetParams) {
            case BOTTOM_CAROUSEL:
                targetPosFinal = slideMotorPosition_ONE_CAROUSEL;
                verticalServoTargetPos = vertServoPosition_ONE_CAROUSEL;
                break;
            case MIDDLE_CAROUSEL:
                targetPosFinal = slideMotorPosition_TWO_CAROUSEL;
                verticalServoTargetPos = vertServoPosition_TWO_CAROUSEL;
                break;
            case TOP_CAROUSEL:
                targetPosFinal = slideMotorPosition_THREE_CAROUSEL;
                verticalServoTargetPos = vertServoPosition_THREE_CAROUSEL;
                break;
            case TOP_DEPOSIT:
                targetPosFinal = slideMotorPosition_THREE_DEPOSIT;
                verticalServoTargetPos = vertServoPosition_THREE_DEPOSIT;
                break;
            case TOP_DEPOSIT_AUTON:
                targetPosFinal = slideMotorPosition_THREE_DEPOSIT_AUTON;
                verticalServoTargetPos = vertServoPosition_THREE_DEPOSIT;
                break;
            case MID_DEPOSIT:
                targetPosFinal = slideMotorPosition_TWO_DEPOSIT;
                verticalServoTargetPos = vertServoPosition_TWO_DEPOSIT;
                break;
            case BOTTOM_DEPOSIT:
                targetPosFinal = slideMotorPosition_ONE_DEPOSIT;
                verticalServoTargetPos = vertServoPosition_ONE_DEPOSIT;
                break;
            case CAP_FROM_CAROUSEL:
                targetPosFinal = slideMotorPosition_CAP_FROM_CAROUSEL;
                verticalServoTargetPos = vertServoPosition_CAP_CAROUSEL_HIGHER;
                break;
            case SHARED_TWO:
                targetPosFinal = slideMotorPosition_SHARED;
                verticalServoTargetPos = vertServoPosition_SHARED_TWO;
                break;
            case SHARED_ONE:
                targetPosFinal = slideMotorPosition_SHARED;
                verticalServoTargetPos = vertServoPosition_SHARED_ONE;
                break;
            case CUSTOM:
                targetPosFinal = slideMotorPosition_CUSTOM;
                verticalServoTargetPos = vertServoPosition_CUSTOM;
                break;
            case NA:
                Log.d("SlideController", "Slide Extension failed: did not specify target. Exiting");
                flagToLeave = true;
                return;
        }
        if (isTeleop) {
            incrementDeltaExtend = incrementDeltaExtendTeleOp;
            incrementDeltaRetract = incrementDeltaRetractTeleop;
        }
        if (targetParams == SlideTarget.CAP_FROM_CAROUSEL) {
            incrementDeltaExtend = incrementDeltaExtendCapstone;
        } else {
            incrementDeltaExtend = incrementDeltaExtendTeleOp;
        }
        blueDumperServo.setPosition(dumperPosition_CLOSED);
        redDumperServo.setPosition(dumperPosition_CLOSED);
        slideMotor.setTargetPosition(targetPosFinal);
        if (targetParams == SlideTarget.CAP_FROM_CAROUSEL) {
            slideMotor.setPower(slideMotorPowerCarousel);
        } else {
            slideMotor.setPower(slideMotorPowerMoving);
        }

        //Wait until the slide bucket is extended outside of the robot
        double TIMER_MAX = 2; // s
        double currentTime = NanoClock.system().seconds();
        double timeToKill = currentTime + TIMER_MAX;
        while (slideMotor.getCurrentPosition() <= slideMotorPosition_BUCKET_OUT + ASlideExtendPositionOffSet) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if(timeToKill <= NanoClock.system().seconds())
            {
                hardStopReset();
            }
        }

        //Now that the bucket is out, start lifting the slide
        //setHeightToParams(verticalServoTargetPos);
        setHeightTo_NewAutonomous(verticalServoTargetPos);
        //verticalServo.setPosition(verticalServoTargetPos);
        //setHeightWithSlope(targetPosFinal, verticalServoTargetPos);

        //Wait until the slide has reached its final position
         TIMER_MAX = 2; // s
         currentTime = NanoClock.system().seconds();
         timeToKill = currentTime + TIMER_MAX;
        while (slideMotor.getCurrentPosition() < targetPosFinal) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if(timeToKill <= NanoClock.system().seconds())
            {
                hardStopReset();
            }

        }
        //Kill the motor's PID and stop so it doesn't try to correct and jitter
        hardStopReset();

        Log.d("SlideController", "slide should be extended");
        slideState = SlideState.OUT_FULLY;
    }

    /**
     * Sets the slide state to retract, does the action for this state (retracts), then sets the state to park.
     **/
    public void retractSlide_NewAutonomous() {
        slideState = SlideState.RETRACTING;

        blueDumperServo.setPosition(dumperPosition_RETRACTING);
        redDumperServo.setPosition(dumperPosition_RETRACTING);
        BlueTeleOp.driveSpeedStrafe = 1;
        slideMotor.setPower(slideMotorPowerMoving);
        slideMotor.setTargetPosition(slideMotorPosition_PARKED);

        double TIMER_MAX = 2; // s
        double currentTime = NanoClock.system().seconds();
        double timeToKill = currentTime + TIMER_MAX;
        //Wait until the slide is retracted to right outside the robot
        while (slideMotor.getCurrentPosition() > slideMotorPosition_BUCKET_OUT_RET + ASlideRetractPositionOffset) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if(timeToKill <= NanoClock.system().seconds())
            {
                hardStopReset();
            }
        }

        //Once right outside, slow down the slide and lower it.
        setHeightTo_NewAutonomous(vertServoPosition_PARKED);
        slideMotor.setPower(slideMotorPowerMovingBack);

        //Wait until the magnetic switch is triggered.
         TIMER_MAX = 2; // s
         currentTime = NanoClock.system().seconds();
         timeToKill = currentTime + TIMER_MAX;
        while (slideMotor.isBusy()) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if(timeToKill <= NanoClock.system().seconds())
            {
                hardStopReset();
            }
            //Once triggered, kill the motor's PID and stop to prevent overshooting and hitting the robot.
            if (!magswitch.getState() || slideMotor.getCurrentPosition()<10) {
                hardStopReset();
                break;
            }
        }

        creepBack_NewAutonomous();
        Log.d("SlideController", "Set the dumper servo to ready (485)");
        blueDumperServo.setPosition(dumperPosition_READY);
        redDumperServo.setPosition(dumperPosition_READY);
        Log.d("SlideController", "Should be inside " + slideMotor.getCurrentPosition() + " " + verticalServo.getPosition());
        slideState = SlideState.PARKED;
    }

    public void setHeightTo_NewAutonomous(double targetPos) {
        double deltaTheta = Math.abs(targetPos - verticalServo.getPosition());
        double runTime = 0.35;
        double initialServoTime = NanoClock.system().seconds();
        double initialServoPosition = verticalServo.getPosition();
        double currentServoPosition = initialServoPosition;
        double deltaTime_s;

        if (verticalServo.getPosition() < targetPos) {  //Move the slide up
            double omegaI = 0.83 * deltaTheta / runTime;
            while (currentServoPosition <= targetPos) {
                deltaTime_s = NanoClock.system().seconds() - initialServoTime;
                verticalServo.setPosition(currentServoPosition);
                currentServoPosition = 0.25 * omegaI * omegaI * deltaTime_s * deltaTime_s / deltaTheta + omegaI * deltaTime_s + initialServoPosition;
            }
        } else {    //Move the slide down
            double omegaI = 2.00 * deltaTheta / runTime;
            while (currentServoPosition >= targetPos) {
                deltaTime_s = NanoClock.system().seconds() - initialServoTime;
                verticalServo.setPosition(currentServoPosition);
                currentServoPosition = -0.25 * omegaI * omegaI * deltaTime_s * deltaTime_s / deltaTheta + initialServoPosition;
            }
        }

        verticalServo.setPosition(targetPos);
    }

    /**
     * Called after a state change in order to change the robot's physical state.
     * Resting states such as PARKED and OUT_FULLY have no associated operations.
     */
    private void doStateAction() {
        int targetPos = 0;
        int targetPosFinal = 0;
        double verticalServoTargetPos = 0;
        switch (slideState) {
            //Resting states, just return.
            case PARKED:
            case OUT_FULLY:
                return;
            case EXTENDING:
                //Switch to set the height and distance of extension depending on the target
                BlueTeleOp.driveSpeedStrafe = Configuration.SlowMovementStrafeMultiplier;
                switch (targetParams) {
                    case BOTTOM_CAROUSEL:
                        targetPosFinal = slideMotorPosition_ONE_CAROUSEL;
                        verticalServoTargetPos = vertServoPosition_ONE_CAROUSEL;
                        break;
                    case MIDDLE_CAROUSEL:
                        targetPosFinal = slideMotorPosition_TWO_CAROUSEL;
                        verticalServoTargetPos = vertServoPosition_TWO_CAROUSEL;
                        break;
                    case TOP_CAROUSEL:
                        targetPosFinal = slideMotorPosition_THREE_CAROUSEL;
                        verticalServoTargetPos = vertServoPosition_THREE_CAROUSEL;
                        break;
                    case TOP_DEPOSIT:
                        targetPosFinal = slideMotorPosition_THREE_DEPOSIT;
                        verticalServoTargetPos = vertServoPosition_THREE_DEPOSIT;
                        break;
                    case TOP_DEPOSIT_AUTON:
                        targetPosFinal = slideMotorPosition_THREE_DEPOSIT_AUTON;
                        verticalServoTargetPos = vertServoPosition_THREE_DEPOSIT;
                        break;
                    case MID_DEPOSIT:
                        targetPosFinal = slideMotorPosition_TWO_DEPOSIT;
                        verticalServoTargetPos = vertServoPosition_TWO_DEPOSIT;
                        break;
                    case BOTTOM_DEPOSIT:
                        targetPosFinal = slideMotorPosition_ONE_DEPOSIT;
                        verticalServoTargetPos = vertServoPosition_ONE_DEPOSIT;
                        break;
                    case CAP_FROM_CAROUSEL:
                        targetPosFinal = slideMotorPosition_CAP_FROM_CAROUSEL;
                        verticalServoTargetPos = vertServoPosition_CAP_CAROUSEL_HIGHER;
                        break;
                    case SHARED_TWO:
                        targetPosFinal = slideMotorPosition_SHARED;
                        verticalServoTargetPos = vertServoPosition_SHARED_TWO;
                        break;
                    case SHARED_ONE:
                        targetPosFinal = slideMotorPosition_SHARED;
                        verticalServoTargetPos = vertServoPosition_SHARED_ONE;
                        break;
                    case CUSTOM:
                        targetPosFinal = slideMotorPosition_CUSTOM;
                        verticalServoTargetPos = vertServoPosition_CUSTOM;
                        break;
                    case NA:
                        Log.d("SlideController", "Slide Extension failed: did not specify target. Exiting");
                        flagToLeave = true;
                        return;
                }
                if (isTeleop) {
                    incrementDeltaExtend = incrementDeltaExtendTeleOp;
                    incrementDeltaRetract = incrementDeltaRetractTeleop;
                }
                if (targetParams == SlideTarget.CAP_FROM_CAROUSEL) {
                    incrementDeltaExtend = incrementDeltaExtendCapstone;
                } else {
                    incrementDeltaExtend = incrementDeltaExtendTeleOp;
                }
                blueDumperServo.setPosition(dumperPosition_CLOSED);
                redDumperServo.setPosition(dumperPosition_CLOSED);
                slideMotor.setTargetPosition(targetPosFinal);
                if (targetParams == SlideTarget.CAP_FROM_CAROUSEL) {
                    slideMotor.setPower(slideMotorPowerCarousel);
                } else {
                    slideMotor.setPower(slideMotorPowerMoving);
                }

                //Wait until the slide bucket is extended outside of the robot
                double TIMER_MAX = 2; // s
                double currentTime = NanoClock.system().seconds();
                double timeToKill = currentTime + TIMER_MAX;
                while (slideMotor.getCurrentPosition() <= slideMotorPosition_BUCKET_OUT) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    if(timeToKill <= NanoClock.system().seconds())
                    {
                        hardStopReset();
                    }
                }
                //Now that the bucket is out, start lifting the slide
                //setHeightToParams(verticalServoTargetPos);
                setHeightToParams(verticalServoTargetPos);
                //verticalServo.setPosition(verticalServoTargetPos);
                //setHeightWithSlope(targetPosFinal, verticalServoTargetPos);

                //Wait until the slide has reached its final position
                 TIMER_MAX = 2; // s
                 currentTime = NanoClock.system().seconds();
                 timeToKill = currentTime + TIMER_MAX;
                while (slideMotor.getCurrentPosition() < targetPosFinal) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    if(timeToKill <= NanoClock.system().seconds())
                    {
                        hardStopReset();
                    }

                }
                //Kill the motor's PID and stop so it doesn't try to correct and jitter
                hardStopReset();


                // if(targetParams == SlideTarget.CAP_FROM_CAROUSEL)
                //{
                //  lower to plop capstone
                //  verticalServoTargetPos = vertServoPosition_CAP_CAROUSEL;
                //setHeightToParams(verticalServoTargetPos); //drop
                //}
                return;
            case RETRACTING:
                blueDumperServo.setPosition(dumperPosition_RETRACTING);
                redDumperServo.setPosition(dumperPosition_RETRACTING);
                BlueTeleOp.driveSpeedStrafe = 1;
                targetPos = slideMotorPosition_PARKED;
                slideMotor.setPower(slideMotorPowerMoving);
                slideMotor.setTargetPosition(targetPos);
                //Wait until the slide is retracted to right outside the robot
                //setHeightWithSlope(slideMotorPosition_PARKED, vertServoPosition_PARKED);
                 TIMER_MAX = 2; // s
                 currentTime = NanoClock.system().seconds();
                 timeToKill = currentTime + TIMER_MAX;
                while (slideMotor.getCurrentPosition() > slideMotorPosition_BUCKET_OUT_RET) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    if(timeToKill <= NanoClock.system().seconds())
                    {
                        hardStopReset();
                    }
                }
                setHeightToParams(vertServoPosition_PARKED);
                //Once right outside, slow down the slide and lower it.
                slideMotor.setPower(slideMotorPowerMovingBack);


                //setHeightToParams(vertServoPosition_PARKED);

                //verticalServo.setPosition(vertServoPosition_PARKED);
                //setHeightWithSlope(slideMotorPosition_PARKED, vertServoPosition_PARKED);
                //Wait until the magnetic switch is triggered.
                TIMER_MAX = 2; // s
                currentTime = NanoClock.system().seconds();
                timeToKill = currentTime + TIMER_MAX;
                while (slideMotor.isBusy()) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    if(timeToKill <= NanoClock.system().seconds())
                    {
                        hardStopReset();
                    }
                    //Once triggered, kill the motor's PID and stop to prevent overshooting and hitting the robot.
                    if (!magswitch.getState()) {
                        hardStopReset();
                        break;
                    }
                }

                creepBack();
                Log.d("SlideController", "Set the dumper servo to ready (485)");
                blueDumperServo.setPosition(dumperPosition_READY);
                redDumperServo.setPosition(dumperPosition_READY);
                return;
        }
    }

    /**
     * Sets state, position and target relative to teleOp (called in teleOp).
     **/
    public void initTeleop() {
        isTeleop = true;
        slideState = SlideState.TELEOP;
        verticalServo.setPosition(vertServoPosition_PARKED);
        this.blueDumperServo.setPosition(dumperPosition_READY);
        this.redDumperServo.setPosition(dumperPosition_READY);
        creepBack();
        hardStopReset();
        resetEncoder();


        this.targetParams = SlideTarget.TOP_DEPOSIT;
    }

    /**
     * Accessor methods for the slide servo and motor.
     **/
    public double getVerticalServoPosition() {
        return verticalServo.getPosition();
    }

    public double getSlideMotorPosition() {
        return slideMotor.getCurrentPosition();
    }

    @Deprecated
    public void setToEncoderPosition(int encoderTicks) {
        int oldPos = slideMotor.getCurrentPosition();
        slideState = SlideState.TELEOP;

        if (slideMotor.isBusy()) {
            return;
        }


        //Checks if the position given is a position that would put the box inside of the robot
        if (encoderTicks > slideMotorPosition_FULL || encoderTicks < slideMotorPosition_PARKED) {
            return;
        }

        if ((encoderTicks < slideMotorPosition_BUCKET_OUT || oldPos < slideMotorPosition_BUCKET_OUT) && (verticalServo.getPosition() < vertServoPosition_PARKED - .01 || verticalServo.getPosition() > vertServoPosition_PARKED + .01)) {
            setHeightToParams(vertServoPosition_PARKED);
        }
        slideMotor.setPower(slideMotorPowerMoving);
        slideMotor.setTargetPosition(encoderTicks);
        while (slideMotor.isBusy()) {
            Sleep.sleep(10);
        }
        slideMotor.setPower(slideMotorPowerStill);
        return;
    }


    /**
     * Sets the slide to a new position given a number of encoder ticks to alter it by.
     *
     * @param encoderTicks the number of ticks to increase or decrease the slide position.
     */
    public void incrementEncoderPosition(int encoderTicks, boolean checkSaftey) {

        slideState = SlideState.TELEOP;

        encoderTicks += slideMotor.getCurrentPosition();
        //Checks if the position given is a position that would put the box inside of the robot
        if ((encoderTicks > slideMotorPosition_FULL || encoderTicks < slideMotorPosition_PARKED) && checkSaftey) {
            return;
        }
        if (!magswitch.getState() && !checkSaftey) {
            resetEncoder();
            return;
        }

        //If the slide is to be incremented inside of the bucket and the slide height is not correct, first set it to the resting position.
        if (encoderTicks < slideMotorPosition_BUCKET_OUT && (verticalServo.getPosition() < vertServoPosition_PARKED - .01 || verticalServo.getPosition() > vertServoPosition_PARKED + .01)) {
            setHeightToParams(vertServoPosition_PARKED);

        }
        slideMotor.setPower(slideMotorPowerMoving);
        slideMotor.setTargetPosition(encoderTicks);
        slideMotor.setPower(slideMotorPowerMoving);

        return;
    }

    /**
     * Sets the slide to a new height given a fraction of the servo's range to alter it by.
     *
     * @param servoPosition A double that represents the servo's range expressed from 0 to 1.
     */
    public void incrementVerticalServo(double servoPosition) {
        servoPosition += verticalServo.getPosition();
        Log.d("Setting Servo Increment To: ", String.valueOf(servoPosition));
        if (servoPosition > vertServoPosition_FULL_MAX) {
            return;
        }
        verticalServo.setPosition(servoPosition);

        return;

    }


    /**
     * Stops any attempted PID correcting by setting the motor's desired position to itself, and resetting the runmode.
     **/
    private void hardStopReset() {
        //STOP!!!!!!!!!!!!
        slideMotor.setPower(0);
        slideMotor.setTargetPosition(slideMotor.getCurrentPosition());
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setPower(0);
        slideMotor.setTargetPosition(slideMotor.getCurrentPosition());
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0);

    }

    private void creepBack() {
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setPower(-.3);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        slideMotor.setPower(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private void creepBack_NewAutonomous() {
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setPower(-.25);
        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        slideMotor.setPower(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoder() {
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    /**  public void checkForBucketObject() {
     Log.d("SlideController", "Found an object in the bucket");
     if (blueColorRangeSensor.alpha() > 160) {
     Log.d("SlideController", "Closing the dumper servo");
     dumperServo.setPosition(dumperPosition_CLOSED);
     }
     } **/
    /**
     * @Deprecated public boolean teleopIsObjectInBucket() {
     * if (blueColorRangeSensor.alpha() > 160) {
     * return true;
     * } else {
     * return false;
     * }
     * }
     **/

    public void setHeightWithSlope(int finalEncoderTicks, double finalServoPos) {
        while (slideMotor.getCurrentPosition() > finalEncoderTicks) {
            changePositionSlope = (0.85 / 1050) * (slideMotor.getCurrentPosition() - 150);
            verticalServo.setPosition(changePositionSlope);
            Log.d("throw", String.valueOf(changePositionSlope));
        }
        verticalServo.setPosition(finalServoPos);
    }

    public static double vertServoPosition_PARKED = 0;//.1
    public static double vertServoPosition_ONE_CAROUSEL = 0.175;
    public static double vertServoPosition_TWO_CAROUSEL = 0.3767; ///measured
    public static double vertServoPosition_THREE_CAROUSEL = 0.647;
    public static double vertServoPosition_THREE_DEPOSIT = .89; // 0.85; // TODO //.754; //measured
    public static double vertServoPosition_TWO_DEPOSIT = 0.44;//.4188;//0.3688;
    public static double vertServoPosition_ONE_DEPOSIT = 0.14;//11;//0.06;
    public static double vertServoPosition_CUSTOM = .6;//11;//0.06;
    public static double vertServoPosition_SHARED_TWO = .447;//11;//0.06;
    public static double vertServoPosition_SHARED_ONE = .227;//11;//0.06;
    public static double vertServoPosition_GRAB_CAP = 0.09; //.09


    double vertServoPosition_PARKED_MIN = 0;
    double vertServoPosition_PARKED_MAX = 0.3;
    public static double vertServoPosition_CAP_CAROUSEL_HIGHER = 1; //TODO
    public static double vertServoPosition_CAP_CAROUSEL = 0.76; // TODO
    double vertServoPosition_FULL_MAX = 1;


    public static double incrementDeltaExtend = 0.05;//.008;
    public static double incrementDeltaRetract = 0.02;//0.007;

    public static double incrementDeltaExtendTeleOp = 0.03;//.008;
    public static double incrementDeltaRetractTeleop = 0.02;//0.007;

    public static double incrementDeltaExtendCapstone = 0.005;
    public static double incrementDeltaRetractCaptsone = 0.002;


    public static double dumperPosition_CLOSED = .53;//0.269;  // remeasured on jan 31 at 16h08
    public static double dumperPosition_READY = .69;//0.55; // 0.69
    public static double dumperPosition_EJECT = .85;//0.74;
    public static double dumperPosition_RETRACTING = .29;//0.05;


    int slideMotorPosition_PARKED = 5;
    public static int slideMotorPosition_BUCKET_OUT = 225;//250;//380//150; // minimum position for the bucket to be out, measured
    public static int slideMotorPosition_BUCKET_OUT_RET = 800; // minimum position for the bucket to be out, measured
    public static int slideMotorPosition_AUTON_EARLY = 400; // minimum position for the bucket to be out, measured
    public static int slideMotorPosition_THREE_DEPOSIT = 1316; // remeasured // last 1310
    public static int slideMotorPosition_THREE_DEPOSIT_AUTON = 1360; // remeasured // last 1360
    public static int slideMotorPosition_TWO_DEPOSIT = 1320; //measured
    public static int slideMotorPosition_ONE_DEPOSIT = 1190;//1000; //measured
    public static int slideMotorPosition_THREE_CAROUSEL = 1713;
    public static int slideMotorPosition_TWO_CAROUSEL = 1600;
    public static int slideMotorPosition_ONE_CAROUSEL = 1665;
    public static int slideMotorPosition_CAP_FROM_CAROUSEL = 1476; // TODO
    public static int slideMotorPosition_CAP_FROM_CAROUSEL_RET = 1442; // TODO
    public static int slideMotorPosition_SHARED = 2030; // TODO
    public static int slideMotorPosition_CUSTOM = 600; // TODO
    public static int slideMotorPosition_FULL = 2050;
    public static int slideMotorPosition_CAP_ON_GROUND = 473;

    public static double slideMotorPowerMoving = 1;
    public static double slideMotorPowerCarousel = .8;
    public static double slideMotorPowerMovingBack = .9;
    public static double slideMotorPowerGrabCap = .9;
    public static final double slideMotorPowerStill = 0;


}
