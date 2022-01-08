package org.sbs.bears.robotframework.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideExtensionController {

    Servo cubeDropper;
    DcMotor slideMotor;

    public SlideExtensionController(HardwareMap hardwareMap, Telemetry telemetry)
    {

        /*this.slideMotor = hardwareMap.get(DcMotor.class,"spool");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.cubeDropper = hardwareMap.get(Servo.class,"du");
        slideMotor.setTargetPosition(slideMotorRetractPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

    }
    public void extendSlide()
    {
        //slideMotor.setTargetPosition(slideMotorExtendPosition);
    }
    public void retractSlide()
    {
        //slideMotor.setTargetPosition(slideMotorRetractPosition);
    }
    public void dropCube()
    {
       /* cubeDropper.setPosition(cubeDropPosition);
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        cubeDropper.setPosition(cubeBackPosition);*/
    }
    public void extendDropRetract()
    {
        extendSlide();
        dropCube();
        retractSlide();
    }

    public static double cubeDropPosition = 0; // TODO
    public static double cubeBackPosition = 0; // TODO
    public static int slideMotorExtendPosition = 0; // TODO
    public static int slideMotorRetractPosition = 0; // TODO
}
