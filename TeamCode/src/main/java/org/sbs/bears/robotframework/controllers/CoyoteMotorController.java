package org.sbs.bears.robotframework.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.sbs.bears.robotframework.Sleep;

@Config
public class CoyoteMotorController {
    DcMotor motor;
    int targetPosition;
    double targetTime;
    public static double POWER_TO_VEL_CONSTANT = 1.0/1000;

    public CoyoteMotorController(HardwareMap hardwareMap, String nameOfMotor)
    {
        motor = hardwareMap.dcMotor.get(nameOfMotor);
        targetPosition = 0;
        targetTime = 0;
    }
    public void goToPositionWithSpeed(int targetPosition, double targetTime)
    {
        this.targetPosition = targetPosition;
        this.targetTime = targetTime;
        double power = calculateMotorPower();
        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        while(motor.isBusy()) {
            Sleep.sleep(10);
        }
    }

    private double calculateMotorPower() {
        // get vel
        double vel = targetPosition/(double) targetTime;
        return vel*POWER_TO_VEL_CONSTANT;
    }
}
