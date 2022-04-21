package org.sbs.bears.Tank;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TapeController {
    private Servo redRotate;
    private Servo redTilt;
    private Servo redExtend;
    private Servo blueRotate;
    private Servo blueTilt;
    private Servo blueExtend;


    private Servo rotate;
    private Servo tilt;
    private Servo extend;

    public TapeController(HardwareMap hardwareMap){
        redRotate = hardwareMap.get(Servo.class, "rtr");
        redTilt = hardwareMap.get(Servo.class, "rtt");
        redExtend = hardwareMap.get(Servo.class, "rte");
        blueRotate = hardwareMap.get(Servo.class, "btr");
        blueTilt = hardwareMap.get(Servo.class, "btt");
        blueExtend = hardwareMap.get(Servo.class, "bte");


        rotate = blueRotate;
        tilt = blueTilt;
        extend = blueExtend;
    }

    public void initServos(){
        redRotate.setPosition(.5);
        redTilt.setPosition(.5);
        blueRotate.setPosition(.5);
        blueTilt.setPosition(.5);

        redExtend.setPosition(.5);
        blueExtend.setPosition(.5);
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
        redExtend.setPosition(power);
    }
    public void rotateBlue(double increment){
        blueRotate.setPosition(blueRotate.getPosition() + increment);
    }
    public void tiltBlue(double increment){
        blueTilt.setPosition(blueTilt.getPosition() + increment);
    }
    public void extendBlue(double power){
        blueExtend.setPosition(power);
    }

    public void rotate(double increment){
        rotate.setPosition(rotate.getPosition() + increment);
    }
    public void tilt(double increment){
        tilt.setPosition(tilt.getPosition() + increment);
    }
    public void extend(double power){
        extend.setPosition(power);
    }

}
