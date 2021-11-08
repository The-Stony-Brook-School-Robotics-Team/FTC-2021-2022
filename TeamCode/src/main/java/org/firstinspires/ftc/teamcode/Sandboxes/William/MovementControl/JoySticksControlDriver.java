package org.firstinspires.ftc.teamcode.Sandboxes.William.MovementControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class JoySticksControlDriver extends OpMode implements Runnable {

    private double motorPower = 0.3;

    /**
     * Set the power limit of the motors.
     * @param motorPower between 0.0-0.3
     */
    public void setMotorPower(double motorPower) {
        this.motorPower = motorPower;
    }

    @Override
    public void run() {
        hardwareMap.get(DcMotor.class, ControllerHelper.leftFrontMotorName).setPower(motorPower * (gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
        hardwareMap.get(DcMotor.class, ControllerHelper.rightFrontMotorName).setPower(motorPower * (gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
        hardwareMap.get(DcMotor.class, ControllerHelper.leftBackMotorName).setPower(motorPower * (gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
        hardwareMap.get(DcMotor.class, ControllerHelper.rightBackMotorName).setPower(motorPower * (gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
