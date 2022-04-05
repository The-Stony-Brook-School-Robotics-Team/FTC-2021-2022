package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous
public class I2Ctest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ModernRoboticsI2cRangeSensor rd = hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"rd");
        rd.setI2cAddress(new I2cAddr(0x28));
        waitForStart();
        while(opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("I2C garbage",rd.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
