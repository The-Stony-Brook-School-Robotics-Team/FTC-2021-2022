package org.sbs.bears.Tank;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


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
