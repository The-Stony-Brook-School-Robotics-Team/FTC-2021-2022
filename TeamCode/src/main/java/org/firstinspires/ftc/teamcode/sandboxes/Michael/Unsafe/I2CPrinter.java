package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class I2CPrinter extends OpMode {
    private ModernRoboticsI2cRangeSensor redIntake;
    private ModernRoboticsI2cRangeSensor blueIntake;
    @Override
    public void init() {
        redIntake = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rd");
        blueIntake = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "bd");
        blueIntake.setI2cAddress(I2cAddr.create8bit(0x26));
    }

    @Override
    public void loop() {
        telemetry.addData("red", redIntake.getDistance(DistanceUnit.MM));
        telemetry.addData("blue", blueIntake.getDistance(DistanceUnit.MM));
        telemetry.update();
    }
}
