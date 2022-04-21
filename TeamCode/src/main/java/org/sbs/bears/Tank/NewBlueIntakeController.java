package org.sbs.bears.Tank;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.sbs.bears.robotframework.enums.IntakeState;

public class NewBlueIntakeController {
    private Servo scooper;
    private Servo claw;
    private DcMotor intakeWheel;
    public ModernRoboticsI2cRangeSensor intakeSensor;
    private ModernRoboticsI2cRangeSensor clawSensor;
    private DcMotor slideMotor;

    public volatile IntakeState state = IntakeState.PARK;
    Object stateMutex = new Object();

    public NewBlueIntakeController(HardwareMap hardwareMap, Servo clawServo, ModernRoboticsI2cRangeSensor clawSensor, DcMotor slideMotor){
        scooper = hardwareMap.get(Servo.class, "bi");
        intakeWheel = hardwareMap.get(DcMotor.class, "bim");
        intakeWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "bd");
        intakeSensor.setI2cAddress(I2cAddr.create8bit(0x30));
        this.claw = clawServo;
        this.clawSensor = clawSensor;
        this.slideMotor = slideMotor;
    }

    public void setState(IntakeState intakeState){
        synchronized(stateMutex){
            state = intakeState;
        }
        applyStateChange();
    }

    public IntakeState getState(){
        return state;
    }
    private void applyStateChange() {
        new Thread(() -> {
            switch (state) {
                case DUMP:
                    scooper.setPosition(IntakeConstants.blueScooperPosition_DUMP);
                    try {
                        Thread.sleep((long) IntakeConstants.waitForScooper);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    intakeWheel.setPower(IntakeConstants.intakePower_DUMP);
                    //TODO: NEW STUFF
                    double timeFlag = NanoClock.system().seconds() + 3.5;
                    while (!isInClaw()) {
                        if (NanoClock.system().seconds() > timeFlag) {
                            setState(IntakeState.PARK);
                            return;
                        }
                        try {
                            Thread.sleep(1);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                    claw.setPosition(SlideConstants.claw_CLOSED);
                    intakeWheel.setPower(0);
                    state = IntakeState.PARK;
                    break;
                case BASE:
                    scooper.setPosition(IntakeConstants.blueScooperPosition_BASE);
                    intakeWheel.setPower(IntakeConstants.intakePower_BASE);
                    claw.setPosition(SlideConstants.claw_IDLE);
                    break;
                case PARK:
                    scooper.setPosition(IntakeConstants.blueScooperPosition_DUMP);
                    intakeWheel.setPower(IntakeConstants.intakePower_PARK);
            }
        }).start();
    }

    public boolean isFreight() {
        double distance = intakeSensor.getDistance(DistanceUnit.MM);
        return distance < IntakeConstants.freightDetectionThreshold && distance != 0;
    }
    public boolean isInClaw(){return clawSensor.getDistance(DistanceUnit.MM) < IntakeConstants.clawFreightDetectionThreshold && slideMotor.getCurrentPosition() < SlideConstants.slideMotorExtensionThreshold;}

    public void tick() {
        if (isFreight() && state == IntakeState.BASE)
            setState(IntakeState.DUMP);
        if (slideMotor.getCurrentPosition() > SlideConstants.slideMotorExtensionThreshold) {
            intakeWheel.setPower(0);}
        else{
            switch(state){
                case DUMP:
                    try {
                        Thread.sleep((long) IntakeConstants.waitForScooper);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    intakeWheel.setPower(IntakeConstants.intakePower_DUMP);
                    break;
                case BASE:
                    intakeWheel.setPower(IntakeConstants.intakePower_BASE);
                    break;
                case PARK:
                    intakeWheel.setPower(0);
            }

        }

    }

}
