package org.sbs.bears.Tank;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TapeController {
    private Servo redRotate;
    private Servo redTilt;
    private CRServo redExtend;
    private Servo blueRotate;
    private Servo blueTilt;
    private CRServo blueExtend;


    private Servo rotate;
    private Servo tilt;
    private CRServo extend;

    public TapeController(HardwareMap hardwareMap){
        redRotate = hardwareMap.get(Servo.class, "rtr");
        redTilt = hardwareMap.get(Servo.class, "rtt");
        redExtend = hardwareMap.get(CRServo.class, "rte");
        blueRotate = hardwareMap.get(Servo.class, "btr");
        blueTilt = hardwareMap.get(Servo.class, "btt");
        blueExtend = hardwareMap.get(CRServo.class, "bte");


        rotate = blueRotate;
        tilt = blueTilt;
        extend = blueExtend;
    }

    public void initServos(){
        redRotate.setPosition(.5);
        redTilt.setPosition(.5);
        blueRotate.setPosition(.5);
        blueTilt.setPosition(.5);

        redExtend.setPower(0);
        blueExtend.setPower(0);
    }

    public void switchTape(){
        if(rotate == redRotate){
            rotate = blueRotate;
            tilt = blueTilt;
            extend = blueExtend;
        }
        else{
            rotate = redRotate;
            tilt = redTilt;
            extend = redExtend;
        }
    }

    public void rotateRed(double increment){
        redRotate.setPosition(redRotate.getPosition() + increment);
    }
    public void tiltRed(double increment){
        redTilt.setPosition(redTilt.getPosition() + increment);
    }
    public void extendRed(double power){
        redExtend.setPower(power);
    }
    public void rotateBlue(double increment){
        blueRotate.setPosition(blueRotate.getPosition() + increment);
    }
    public void tiltBlue(double increment){
        blueTilt.setPosition(blueTilt.getPosition() + increment);
    }
    public void extendBlue(double power){
        blueExtend.setPower(power);
    }

    public void rotate(double increment){
        rotate.setPosition(rotate.getPosition() + increment);
    }
    public void tilt(double increment){
        tilt.setPosition(tilt.getPosition() + increment);
    }
    public void extend(double power){
        extend.setPower(power);
    }

}
