package org.sbs.bears.Tank;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

public class NewSlideController {

    public DcMotorEx liftMotor;
    private Servo claw;
    private Servo flipperOne;
    private Servo flipperTwo;
    private DcMotorEx slideMotor;
    private DigitalChannel magswitch;
    private ModernRoboticsI2cRangeSensor clawDistanceSensor;
    private AnalogInput potentiometer;

    private double desiredVoltage;

    public NewSlideController(HardwareMap hardwareMap){
        magswitch = hardwareMap.get(DigitalChannel.class, "mag");

        liftMotor = hardwareMap.get(DcMotorEx.class, "lift");
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //TODO tolerance?
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        claw = hardwareMap.get(Servo.class, "cl");
        clawDistanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "cd");
        clawDistanceSensor.setI2cAddress(I2cAddr.create8bit(0x24));
        flipperOne = hardwareMap.get(Servo.class, "fl");
        //flipperOne.setPosition(SlideConstants.flipper_READY);
        flipperOne.setDirection(Servo.Direction.REVERSE);
        flipperTwo = hardwareMap.get(Servo.class, "fl2");

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
        potentiometer = hardwareMap.get(AnalogInput.class, "po");

        creepBack();
        setTargetHeight(SlideConstants.potentiometer_THREE_DEPOSIT);

    }

    public void setTargetHeight(double desiredVoltage){

        this.desiredVoltage = desiredVoltage;
        waitForSetHeight.start();

    }
    public void extend(int targetPosition, double flipperPosition, double potentiometerPosition){
        slideMotor.setPower(SlideConstants.slideMotorPower_EXTENDING);
        slideMotor.setTargetPosition(targetPosition);
        flipperOne.setPosition(flipperPosition);
        flipperTwo.setPosition(flipperPosition - SlideConstants.flipperOffset);
    }
    public void retract(){
        slideMotor.setPower(SlideConstants.slideMotorPower_RETRACTING);
        slideMotor.setTargetPosition(SlideConstants.slideMotorPosition_PARKED);
        flipperOne.setPosition(SlideConstants.flipper_READY);
        flipperTwo.setPosition(SlideConstants.flipper_READY - SlideConstants.flipperOffset);
        waitToCreep.start();
    }
    //Handle threading externally if need be.
    public void dropFreight(){
        if(!dropFreightThread.isAlive()){
            dropFreightThread.start();
        }
    }
    public void extendDropRetract(int targetPosition, double flipperPosition, double potentiometerPosition){
        extend(targetPosition, flipperPosition, potentiometerPosition);
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
        waitForSetHeight.interrupt();
        dropFreightThread.interrupt();
    }

    public Servo getClaw(){return claw;}
    public ModernRoboticsI2cRangeSensor getDistanceSensor(){return clawDistanceSensor;}
    public DcMotor getSlideMotor(){return slideMotor;}

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
            while(slideMotor.isBusy()) {
                try {
                    Thread.sleep(SlideConstants.busyWait);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            creepBack();
        }
    };

    Thread waitForSetHeight = new Thread(){
        public void run(){
          if(desiredVoltage < SlideConstants.potentiometer_IF_LOWER_THAN || desiredVoltage > SlideConstants.potentiometer_IF_HIGHER_THAN){
                liftMotor.setPower(0);
              return;
          }
            if(potentiometer.getVoltage() < desiredVoltage) {
                while (potentiometer.getVoltage() < desiredVoltage) {
                    liftMotor.setPower(SlideConstants.liftMotorPower_MOVING);
                }
                liftMotor.setPower(0);
                return;
            }
            while (potentiometer.getVoltage() > desiredVoltage){
                liftMotor.setPower(-SlideConstants.liftMotorPower_MOVING);
            }
            liftMotor.setPower(0);

        }
    };
    Thread dropFreightThread = new Thread(){
        public void run(){
            claw.setPosition(SlideConstants.claw_OPEN);
            try {
                Thread.sleep(600);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            claw.setPosition(SlideConstants.claw_IDLE);
        }
    };
}
