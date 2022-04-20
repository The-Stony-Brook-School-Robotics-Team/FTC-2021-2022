package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "drive controller test")
public class DriveControllerTest extends OpMode {
    private DriveController driveController;
    public static double ticks = 800;
    @Override
    public void init() {
    driveController = new DriveController(hardwareMap);

    }

    @Override
    public void start(){
        driveController.driveTo((int)ticks);

    }

    @Override
    public void loop() {
       driveController.update();


    }
}
