package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.sbs.bears.robotframework.enums.IntakeState;

public class NewIntakeController {
    private Servo scooper;
    private Servo claw;
    private DcMotor intakeWheel;
    public ModernRoboticsI2cRangeSensor intakeSensor;
    private ModernRoboticsI2cRangeSensor clawSensor;

    private int freightDetectionThreshold = 50;

    public volatile IntakeState state = IntakeState.DUMP;
    Object stateMutex = new Object();

    public NewIntakeController(HardwareMap hardwareMap, Servo clawServo, ModernRoboticsI2cRangeSensor clawSensor){
        scooper = hardwareMap.get(Servo.class, "ri");
        intakeWheel = hardwareMap.get(DcMotor.class, "rim");
        intakeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rd");
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
                    scooper.setPosition(IntakeConstants.scooperPosition_DUMP);
                    claw.setPosition(IntakeConstants.clawPosition_DUMP);
                    intakeWheel.setPower(IntakeConstants.intakePower_DUMP);
                    break;
                case BASE:
                    scooper.setPosition(IntakeConstants.scooperPosition_BASE);
                    claw.setPosition(IntakeConstants.clawPosition_BASE);
                    intakeWheel.setPower(IntakeConstants.intakePower_BASE);
                    break;

            }
        }).start();
    }

    public boolean isFreight(){return intakeSensor.getDistance(DistanceUnit.MM) < freightDetectionThreshold;}

    public void tick(){
        if(isFreight()) setState(IntakeState.DUMP);
    }

}
