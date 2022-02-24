package org.firstinspires.ftc.teamcode.common.newAutonomous;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop;
import org.sbs.bears.robotframework.Beta;
import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.enums.SlideState;
import org.sbs.bears.robotframework.enums.SlideTarget;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

@Config
public class AutonomousSlideController {
    public Servo verticalServo;
    public Servo dumperServo;
    public DigitalChannel magSwitch;

    public DcMotorEx slideMotor;

    public SlideState slideState = SlideState.PARKED;
    public SlideTarget targetParams = SlideTarget.NA;
    private boolean isTeleOp;

    public static double SERVO_VELOCITY_CONSTANT = 0.8;
    public static boolean SERVO_TEST = false;

    public volatile static boolean next = false;

    public static int finishLiftingSlideEncoderPosition = -1;

    //TODO: Put back at the bottom or michael will kill me
    public static double vertServoPosition_GRAB_CAP = 0.09; //.09

    public AutonomousSlideController(HardwareMap hardwareMap, Telemetry telemetry) {
        magSwitch = hardwareMap.get(DigitalChannel.class, "stop");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);
        verticalServo = hardwareMap.get(Servo.class, "vt");
        dumperServo = hardwareMap.get(Servo.class, "du");
        slideMotor = hardwareMap.get(DcMotorEx.class, "spool");

        //TODO:------------------------------------------------------
        slideMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(10, 0, 0, 0));
        //TODO:------------------------------------------------------

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(0); // should be where it reset to
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dumperServo.setPosition(dumperPosition_READY);
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
     * Sets state, position and target relative to teleOp (called in teleOp).
     **/
    public void initTeleOp() {
        isTeleOp = true;
        slideState = SlideState.TELEOP;
        verticalServo.setPosition(vertServoPosition_PARKED);
        this.dumperServo.setPosition(dumperPosition_READY);


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
        slideMotor.setPower(SLIDE_MOTOR_POWER_MOVING);
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
        if (!magSwitch.getState() && !checkSaftey) {
            resetEncoder();
            return;
        }

        //If the slide is to be incremented inside of the bucket and the slide height is not correct, first set it to the resting position.
        if (encoderTicks < slideMotorPosition_BUCKET_OUT && (verticalServo.getPosition() < vertServoPosition_PARKED - .01 || verticalServo.getPosition() > vertServoPosition_PARKED + .01)) {
            setHeightToParams(vertServoPosition_PARKED);

        }
        slideMotor.setPower(SLIDE_MOTOR_POWER_MOVING);
        slideMotor.setTargetPosition(encoderTicks);
        slideMotor.setPower(SLIDE_MOTOR_POWER_MOVING);

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
        //setHeightToParams(servoPosition);
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

    public void resetEncoder() {
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void extendDropRetract_Autonomous(SlideTarget slideTarget) {
        extendSlide_Autonomous(slideTarget);
        dropCube_Autonomous();
        retractSlide_Autonomous_ThreadEdition();
        this.targetParams = SlideTarget.NA;
    }

    public void extendSlide_Autonomous(SlideTarget slideTarget) {
        int slideMotorTargetPosition = 0;
        double verticalServoTargetPosition = 0;
        OfficialTeleop.driveSpeedStrafe = Configuration.SlowMovementStrafeMultiplier;

        switch (slideTarget) {
            case BOTTOM_CAROUSEL:
                slideMotorTargetPosition = slideMotorPosition_ONE_CAROUSEL;
                verticalServoTargetPosition = vertServoPosition_ONE_CAROUSEL;
                break;
            case MIDDLE_CAROUSEL:
                slideMotorTargetPosition = slideMotorPosition_TWO_CAROUSEL;
                verticalServoTargetPosition = vertServoPosition_TWO_CAROUSEL;
                break;
            case TOP_CAROUSEL:
                slideMotorTargetPosition = slideMotorPosition_THREE_CAROUSEL;
                verticalServoTargetPosition = vertServoPosition_THREE_CAROUSEL;
                break;
            case TOP_DEPOSIT:
                slideMotorTargetPosition = slideMotorPosition_THREE_DEPOSIT;
                verticalServoTargetPosition = vertServoPosition_THREE_DEPOSIT;
                break;
            case TOP_DEPOSIT_AUTON:
                slideMotorTargetPosition = slideMotorPosition_THREE_DEPOSIT_AUTON;
                verticalServoTargetPosition = vertServoPosition_THREE_DEPOSIT;
                break;
            case MID_DEPOSIT:
                slideMotorTargetPosition = slideMotorPosition_TWO_DEPOSIT;
                verticalServoTargetPosition = vertServoPosition_TWO_DEPOSIT;
                break;
            case BOTTOM_DEPOSIT:
                slideMotorTargetPosition = slideMotorPosition_ONE_DEPOSIT;
                verticalServoTargetPosition = vertServoPosition_ONE_DEPOSIT;
                break;
            case CAP_FROM_CAROUSEL:
                slideMotorTargetPosition = slideMotorPosition_CAP_FROM_CAROUSEL;
                verticalServoTargetPosition = vertServoPosition_CAP_CAROUSEL_HIGHER;
                break;
            case CUSTOM:
                slideMotorTargetPosition = slideMotorPosition_CUSTOM;
                verticalServoTargetPosition = vertServoPosition_CUSTOM;
                break;
        }

        //TODO:-----------------------------------------------
        if (isTeleOp) {
            incrementDeltaExtend = incrementDeltaExtendTeleOp;
            incrementDeltaRetract = incrementDeltaRetractTeleop;
        }
        if (slideTarget == SlideTarget.CAP_FROM_CAROUSEL) {
            incrementDeltaExtend = incrementDeltaExtendCapstone;
        } else {
            incrementDeltaExtend = incrementDeltaExtendTeleOp;
        }
        //TODO:-----------------------------------------------

        //Close the dumper servo and extend the slide bucket to the outside.
        dumperServo.setPosition(dumperPosition_CLOSED);

        //TODO:-----------------------------------------------
        slideMotor.setTargetPosition(slideMotorTargetPosition);   //???
        if (slideTarget == SlideTarget.CAP_FROM_CAROUSEL) {
            slideMotor.setPower(slideMotorPowerCarousel);
        } else {
            slideMotor.setPower(SLIDE_MOTOR_POWER_MOVING);
        }
        //TODO:-----------------------------------------------

        while (slideMotor.getCurrentPosition() <= slideMotorPosition_BUCKET_OUT) {
            try {
                Thread.sleep(30);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        //Now that the bucket is out, start lifting the slide
        setHeightTo_Autonomous(verticalServoTargetPosition);

        //Wait until the slide has reached its final position
        while (slideMotor.getCurrentPosition() < slideMotorTargetPosition) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        hardStopReset();    //<<<<<<<<<<<<<<<<<<<<<<<<<

        return;
    }

    public void dropCube_Autonomous() {
        if (targetParams == SlideTarget.CAP_FROM_CAROUSEL) {
            incrementDeltaRetract = incrementDeltaRetractCaptsone;
            setHeightTo_Autonomous(vertServoPosition_CAP_CAROUSEL);
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
            dumperServo.setPosition(dumperPosition_EJECT);
            try {
                Thread.sleep(260);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        dumperServo.setPosition(dumperPosition_RETRACTING);
    }

    Thread lowerSlideAndBringItBackThread = new Thread(() -> {
        //Once right outside, slow down the slide and lower it.
        setHeightTo_Autonomous(vertServoPosition_PARKED);

        slideMotor.setPower(slideMotorPowerMovingBack);
        while (slideMotor.isBusy()) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //Once triggered, kill the motor's PID and stop to prevent overshooting and hitting the robot.
            if (!magSwitch.getState()) {
                hardStopReset();
                break;
            }
        }

        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setPower(-.3);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        slideMotor.setPower(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dumperServo.setPosition(dumperPosition_READY);
    });

    public void retractSlide_Autonomous_ThreadEdition() {
        dumperServo.setPosition(dumperPosition_RETRACTING);
        OfficialTeleop.driveSpeedStrafe = 1;
        slideMotor.setTargetPosition(slideMotorPosition_PARKED);
        slideMotor.setPower(SLIDE_MOTOR_POWER_MOVING);

        //Wait until the slide is retracted to right outside the robot
        while (slideMotor.getCurrentPosition() > finishLiftingSlideEncoderPosition) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        lowerSlideAndBringItBackThread.start();
    }

    //TODO: ???
    public void retractSlide_Autonomous() {
        dumperServo.setPosition(dumperPosition_RETRACTING);
        OfficialTeleop.driveSpeedStrafe = 1;
        slideMotor.setTargetPosition(slideMotorPosition_PARKED);
        slideMotor.setPower(SLIDE_MOTOR_POWER_MOVING);

        //Wait until the slide is retracted to right outside the robot
        while (slideMotor.getCurrentPosition() > slideMotorPosition_BUCKET_OUT_RET) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        setHeightTo_Autonomous(vertServoPosition_PARKED);

        //Once right outside, slow down the slide and lower it.
        slideMotor.setPower(slideMotorPowerMovingBack);
        while (slideMotor.isBusy()) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //Once triggered, kill the motor's PID and stop to prevent overshooting and hitting the robot.
            if (!magSwitch.getState()) {
                hardStopReset();
                break;
            }
        }

        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setPower(-.3);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        slideMotor.setPower(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dumperServo.setPosition(dumperPosition_READY);
        return;
    }

    public void setHeightTo_Autonomous(double targetPos) {
        double deltaTheta = Math.abs(targetPos - verticalServo.getPosition());
        double runTime = 0.55;
        double omegaI = 2.0 * deltaTheta / runTime;
        double initialServoTime = NanoClock.system().seconds();
        double initialServoPosition = verticalServo.getPosition();
        double currentServoPosition = initialServoPosition;
        double deltaTime_s;

        if (slideState != SlideState.PARKED) {
            if (verticalServo.getPosition() < targetPos) {  //Move the slide up
                while (currentServoPosition <= targetPos) {
                    deltaTime_s = NanoClock.system().seconds() - initialServoTime;
                    verticalServo.setPosition(currentServoPosition);
                    currentServoPosition = 0.25 * omegaI * omegaI * deltaTheta * deltaTime_s / deltaTheta + omegaI * deltaTime_s + initialServoPosition;
                }
                if (finishLiftingSlideEncoderPosition == -1) {
                    finishLiftingSlideEncoderPosition = slideMotor.getCurrentPosition();
                }
            } else {    //Move the slide down
                while (currentServoPosition >= targetPos) {
                    deltaTime_s = NanoClock.system().seconds() - initialServoTime;
                    verticalServo.setPosition(currentServoPosition);
                    currentServoPosition = -0.25 * omegaI * omegaI * deltaTheta * deltaTime_s / deltaTheta + initialServoPosition;
                }
            }

            verticalServo.setPosition(targetPos);
        }
    }


    // TODO MEASURE ALL CONSTANTS

    public static double vertServoPosition_PARKED = 0;//.1
    public static double vertServoPosition_ONE_CAROUSEL = 0.175;
    public static double vertServoPosition_TWO_CAROUSEL = 0.3767; ///measured
    public static double vertServoPosition_THREE_CAROUSEL = 0.647;
    public static double vertServoPosition_THREE_DEPOSIT = .89; // 0.85; // TODO //.754; //measured
    public static double vertServoPosition_TWO_DEPOSIT = 0.44;//.4188;//0.3688;
    public static double vertServoPosition_ONE_DEPOSIT = 0.14;//11;//0.06;
    public static double vertServoPosition_CUSTOM = .6;//11;//0.06;


    double vertServoPosition_PARKED_MIN = 0;
    double vertServoPosition_PARKED_MAX = 0.3;
    public static double vertServoPosition_CAP_CAROUSEL_HIGHER = 1; //TODO
    public static double vertServoPosition_CAP_CAROUSEL = 0.76; // TODO
    double vertServoPosition_FULL_MAX = 1;


    public static double incrementDeltaExtend = 0.03;//.008;
    public static double incrementDeltaRetract = 0.02;//0.007;

    public static double incrementDeltaExtendTeleOp = 0.03;//.008;
    public static double incrementDeltaRetractTeleop = 0.02;//0.007;

    public static double incrementDeltaExtendCapstone = 0.005;
    public static double incrementDeltaRetractCaptsone = 0.002;


    // dumper servo
/*    double dumperPosition_DUMP = .91;
    double dumperPosition_HOLDBLOCK = 0;
*/    public static double dumperPosition_CLOSED = 0.269;  // remeasured on jan 31 at 16h08
    public static double dumperPosition_READY = 0.55;
    public static double dumperPosition_EJECT = 0.74;
    public static double dumperPosition_RETRACTING = 0.05;

    // slide motor
    int slideMotorPosition_PARKED = 5;
    public static int slideMotorPosition_BUCKET_OUT = 225;//250;//380//150; // minimum position for the bucket to be out, measured
    public static int slideMotorPosition_BUCKET_OUT_RET = 800; // minimum position for the bucket to be out, measured
    public static int slideMotorPosition_THREE_DEPOSIT = 1316; // remeasured // last 1310
    public static int slideMotorPosition_THREE_DEPOSIT_AUTON = 1400; // remeasured // last 1360
    public static int slideMotorPosition_TWO_DEPOSIT = 1390; //measured
    public static int slideMotorPosition_ONE_DEPOSIT = 1190;//1000; //measured
    public static int slideMotorPosition_THREE_CAROUSEL = 1713;
    public static int slideMotorPosition_TWO_CAROUSEL = 1650;
    public static int slideMotorPosition_ONE_CAROUSEL = 1665;
    public static int slideMotorPosition_CAP_FROM_CAROUSEL = 1476; // TODO
    public static int slideMotorPosition_CAP_FROM_CAROUSEL_RET = 1442; // TODO
    public static int slideMotorPosition_CUSTOM = 600; // TODO
    public static int slideMotorPosition_FULL = 1980;
    //int slideMotorPosition_START_LOWER = 400;
    public static int slideMotorPosition_CAP_ON_GROUND = 473;

    public static double SLIDE_MOTOR_POWER_MOVING = .9;
    public static double slideMotorPowerCarousel = .8;
    public static double slideMotorPowerMovingBack = .9;
    public static double slideMotorPowerGrabCap = .9;
    public static final double slideMotorPowerStill = 0;

    /*double deltaZForLevel3 = 12; // in
    double deltaZForLevel2 = 5; // in
    double deltaZForLevel1 = 0; // in

    static final Vector2d positionOfBlueHub = new Vector2d(24,12);
*/


}