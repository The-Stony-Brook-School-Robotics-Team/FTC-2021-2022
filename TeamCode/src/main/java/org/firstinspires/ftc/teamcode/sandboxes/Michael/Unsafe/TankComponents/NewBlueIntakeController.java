package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.sbs.bears.robotframework.enums.IntakeState;

public class NewBlueIntakeController {
    private Servo scooper;
    private Servo claw;
    private DcMotor intakeWheel;
    public ModernRoboticsI2cRangeSensor intakeSensor;
    private ModernRoboticsI2cRangeSensor clawSensor;

    public volatile IntakeState state = IntakeState.PARK;
    Object stateMutex = new Object();

    public NewBlueIntakeController(HardwareMap hardwareMap, Servo clawServo, ModernRoboticsI2cRangeSensor clawSensor){
        scooper = hardwareMap.get(Servo.class, "bi");
        intakeWheel = hardwareMap.get(DcMotor.class, "bim");
        intakeWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "bd");
        this.claw = clawServo;
        this.clawSensor = clawSensor;
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
    private void applyStateChange(){
        new Thread(() -> {
            switch(state){
                case DUMP:
                    scooper.setPosition(IntakeConstants.blueScooperPosition_DUMP);
                    try {
                        Thread.sleep((long)IntakeConstants.waitForScooper);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    intakeWheel.setPower(IntakeConstants.intakePower_DUMP);
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
    public boolean isInClaw(){return clawSensor.getDistance(DistanceUnit.MM) < IntakeConstants.clawFreightDetectionThreshold;}

    public void tick(){
        if(isFreight() && state == IntakeState.BASE) setState(IntakeState.DUMP);
        if(isInClaw() && state == IntakeState.DUMP){
            claw.setPosition(SlideConstants.claw_CLOSED);
            intakeWheel.setPower(0);
        }
        else{
            switch(state){
                case DUMP:
                    try {
                        Thread.sleep((long)IntakeConstants.waitForScooper);
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
