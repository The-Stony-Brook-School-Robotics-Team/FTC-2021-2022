package org.firstinspires.ftc.teamcode.common.newAutonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.enums.SlideTarget;

//@Config
public class AutonomousSlideController {
    final boolean isDebug = true;

    public SampleMecanumDrive roadRunnerDrive;

    public Servo verticalServo;
    public Servo dumperServo;
    public DigitalChannel magSwitch;

    public DcMotorEx slideMotor;

    public SlideTarget targetParams = SlideTarget.NA;
    private boolean isTeleOp;

    public static double SERVO_VELOCITY_CONSTANT = 0.8;
    public static boolean SERVO_TEST = false;

    public volatile static boolean next = false;

    public static int finishLiftingSlideEncoderPosition = -1;

    //TODO: Put back at the bottom or michael will kill me
    public static double vertServoPosition_GRAB_CAP = 0.09; //.09

    public AutonomousSlideController(HardwareMap hardwareMap, SampleMecanumDrive roadRunnerDrive) {
        magSwitch = hardwareMap.get(DigitalChannel.class, "stop");
        magSwitch.setMode(DigitalChannel.Mode.INPUT);
        verticalServo = hardwareMap.get(Servo.class, "vt");
        dumperServo = hardwareMap.get(Servo.class, "du");
        slideMotor = hardwareMap.get(DcMotorEx.class, "spool");
        this.roadRunnerDrive = roadRunnerDrive;

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
    }

    /**
     * Accessor methods for the slide servo and motor.
     **/
    public double getVerticalServoPosition() {
        return verticalServo.getPosition();
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

    public Thread extendDropRetract(SlideTarget slideTarget) {
        extendSlide_Autonomous(slideTarget);
        dropCube_Autonomous();
        return retractSlide_Autonomous();
    }

    private volatile boolean needExtendDropRetract = false;
    private Thread currentExtendDropRetractThread;

    public void startExtendDropRetractThread() {
        needExtendDropRetract = true;
    }

    /**
     * You have to send signal to it to start extending.
     *
     * @param slideTarget
     * @return
     */
    public Thread initializeExtendDropRetractThread(SlideTarget slideTarget) {
        if (!needExtendDropRetract) {
            currentExtendDropRetractThread.interrupt();
        }

        try {
            currentExtendDropRetractThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        needExtendDropRetract = false;

        currentExtendDropRetractThread = new Thread(() -> {
            while (!needExtendDropRetract && roadRunnerDrive.getPoseEstimate().getX() < 40) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            extendSlide_Autonomous(slideTarget);
            dropCube_Autonomous();
            retractSlide_Autonomous();
        });
        currentExtendDropRetractThread.start();
        return currentExtendDropRetractThread;
    }

    public void extendSlide_Autonomous(SlideTarget slideTarget) {
        int slideMotorTargetPosition = 0;
        double verticalServoTargetPosition = 0;

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
        slideMotor.setTargetPosition(slideMotorTargetPosition);
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

    private Thread getLowerSlideAndBringItBackThread() {
        return new Thread(() -> {
            if (!isDebug) {
                while (slideMotor.getCurrentPosition() > finishLiftingSlideEncoderPosition) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            } else {
                System.out.println(finishLiftingSlideEncoderPosition);
            }

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
    }

    /**
     * Retract slide in a thread.
     *
     * @return The thread that brings the slide back.
     */
    public Thread retractSlide_Autonomous() {
        Thread lowerSlideAndBringItBackThread = getLowerSlideAndBringItBackThread();
        dumperServo.setPosition(dumperPosition_RETRACTING);
        slideMotor.setTargetPosition(slideMotorPosition_PARKED);
        slideMotor.setPower(SLIDE_MOTOR_POWER_MOVING);
        lowerSlideAndBringItBackThread.start();
        try {
            lowerSlideAndBringItBackThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return lowerSlideAndBringItBackThread;
    }

    public void setHeightTo_Autonomous(double targetPos) {
        double deltaTheta = Math.abs(targetPos - verticalServo.getPosition());
        double runTime = 0.55;
        double initialServoTime = NanoClock.system().seconds();
        double initialServoPosition = verticalServo.getPosition();
        double currentServoPosition = initialServoPosition;
        double deltaTime_s;

        if (verticalServo.getPosition() < targetPos) {  //Move the slide up
            double omegaI = 0.798 * deltaTheta / runTime;
            while (currentServoPosition <= targetPos) {
                deltaTime_s = NanoClock.system().seconds() - initialServoTime;
                verticalServo.setPosition(currentServoPosition);
                currentServoPosition = 0.25 * omegaI * omegaI * deltaTime_s * deltaTime_s / deltaTheta + omegaI * deltaTime_s + initialServoPosition;
            }
            if (finishLiftingSlideEncoderPosition == -1) {
                finishLiftingSlideEncoderPosition = slideMotor.getCurrentPosition();
            }
        } else {    //Move the slide down
            double omegaI = 1.773 * deltaTheta / runTime;
            while (currentServoPosition >= targetPos) {
                deltaTime_s = NanoClock.system().seconds() - initialServoTime;
                verticalServo.setPosition(currentServoPosition);
                currentServoPosition = -0.25 * omegaI * omegaI * deltaTime_s * deltaTime_s / deltaTheta + initialServoPosition;
            }
        }
        verticalServo.setPosition(targetPos);
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