package org.sbs.bears.robotframework.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.robotframework.enums.SlideComponents;

public class SlideController {
    private Servo verticalServo;
    private Servo horizontalServo;
    private Servo dumperServo;

    private DcMotor slideMotor;

    public SlideController(HardwareMap hardwareMap, Telemetry telemetry)
    {
        verticalServo = hardwareMap.get(Servo.class, "vt");
        horizontalServo = hardwareMap.get(Servo.class, "hz");
        dumperServo = hardwareMap.get(Servo.class, "du");
        slideMotor = hardwareMap.get(DcMotor.class, "spool");


        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(0); // should be where it reset to
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // TODO MEASURE ALL CONSTANTS

    // vertical servo
    double vertServoPosition_PARKED = 0;
    double vertServoPosition_ONE_CAROUSEL = 0;
    double vertServoPosition_TWO_CAROUSEL = 0;
    double vertServoPosition_THREE_CAROUSEL = 0;
    double vertServoPosition_THREE_DEPOSIT = 0;
    double incrementDelta = 0.001;

    // dumper servo
    double dumperPosition_DUMP = 0;
    double dumperPosition_HOLDBLOCK = 0;

    // slide motor
    double slideMotorPosition_PARKED = 0;
    double slideMotorPosition_BUCKET_OUT = 0;
    double slideMotorPosition_THREE_DEPOSIT = 0;
    double slideMotorPosition_THREE_CAROUSEL = 0;
    double slideMotorPosition_TWO_CAROUSEL = 0;
    double slideMotorPosition_ONE_CAROUSEL = 0;










}
