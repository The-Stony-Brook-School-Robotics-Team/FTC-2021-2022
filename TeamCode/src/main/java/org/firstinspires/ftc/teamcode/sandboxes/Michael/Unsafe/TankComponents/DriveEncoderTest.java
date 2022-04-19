package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name = "drive encoder test")
public class DriveEncoderTest extends OpMode {
    private DriveController driveController;
    @Override
    public void init() {
        driveController = new DriveController(hardwareMap);
    }

    @Override
    public void start(){
        driveController.driveTo((int)DriveController.inchesToEncoderTicks(3));




    }

    @Override
    public void loop() {


    }

}
