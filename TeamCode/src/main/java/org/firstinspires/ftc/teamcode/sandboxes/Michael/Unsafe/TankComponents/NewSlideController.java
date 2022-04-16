package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class NewSlideController {

    private DcMotorEx liftMotor;
    private Servo claw;
    private Servo flipper;
    private DcMotorEx slideMotor;
    private DigitalChannel magswitch;
    private ModernRoboticsI2cRangeSensor clawDistanceSensor;

    public NewSlideController(HardwareMap hardwareMap){
        magswitch = hardwareMap.get(DigitalChannel.class, "mag");
        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPositionPIDFCoefficients(10);
        //TODO tolerance?
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "cl");
        clawDistanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "cd");

        flipper = hardwareMap.get(Servo.class, "fl");
        flipper.setPosition(SlideConstants.flipper_READY);

        slideMotor = hardwareMap.get(DcMotorEx.class, "spool");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPositionPIDFCoefficients(10);
        slideMotor.setTargetPositionTolerance(SlideConstants.slideMotorTolerance); //TODO
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //claw.setPosition(SlideConstants.frontBucket_CLOSED);
        magswitch.setMode(DigitalChannel.Mode.INPUT);
        //setTargetHeight(SlideConstants.liftServo_THREE_DEPOSIT);

        creepBack();


    }

    public void setTargetHeight(int targetHeight){
        liftMotor.setTargetPosition(targetHeight);
    }
    public void extend(int targetPosition){
        slideMotor.setPower(SlideConstants.slideMotorPower_EXTENDING);
        slideMotor.setTargetPosition(targetPosition);
    }
    public void retract(){
        slideMotor.setPower(SlideConstants.slideMotorPower_RETRACTING);
        slideMotor.setTargetPosition(SlideConstants.slideMotorPosition_PARKED);
        waitToCreep.start();
    }
    //Handle threading externally if need be.
    public void dropFreight(){
        claw.setPosition(SlideConstants.claw_OPEN);
        try {
             Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        claw.setPosition(SlideConstants.claw_CLOSED);
    }
    public void extendDropRetract(int targetPosition){
        extend(targetPosition);
        waitForDropRetract.start();
    }
    private void creepBack() {
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setPower(-0.3);
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        slideMotor.setPower(SlideConstants.slideMotorPower_STILL);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setTargetPositionTolerance(SlideConstants.slideMotorTolerance);
    }
    public void killThreads(){
        waitForDropRetract.interrupt();
        waitToCreep.interrupt();
    }

    public Servo getClaw(){return claw;}
    public ModernRoboticsI2cRangeSensor getDistanceSensor(){return clawDistanceSensor;}

    public boolean isExtendedPastThreshold(){return slideMotor.getCurrentPosition() > SlideConstants.slideMotorExtensionThreshold;}

    public boolean magTriggered(){return !magswitch.getState();}

    Thread waitForDropRetract = new Thread(){
        public void run(){
            while(slideMotor.isBusy()) {
                try {
                    Thread.sleep(SlideConstants.busyWait);
                    //wait();?
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            //notifyAll(); ?
            dropFreight();
            retract();
        }
    };

    Thread waitToCreep = new Thread(){
        public void run(){
            while(slideMotor.isBusy() && !magTriggered()) {
                try {
                    Thread.sleep(SlideConstants.busyWait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            creepBack();
        }
    };

}
