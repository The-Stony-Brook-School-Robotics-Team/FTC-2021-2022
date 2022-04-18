package org.firstinspires.ftc.teamcode.common.official.teleop.v2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.official.teleop.v2.enums.TeleOpStatus;
import org.firstinspires.ftc.teamcode.common.official.teleop.v2.runtimes.DriverRuntime;
import org.firstinspires.ftc.teamcode.common.official.teleop.v2.runtimes.GamepadRuntime;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.NewIntakeController;
import org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents.NewSlideController;

/**
 * Personal Notes -
 * @version 2.0
 * @type lightweight
 * @revisions redone trees
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "A - v2 teleop")
public class TeleOp extends OpMode {

    // status
    private TeleOpStatus status = TeleOpStatus.paused;

    // ui
    private GamepadRuntime primary;
    private GamepadRuntime secondary;
    private DriverRuntime driverRuntime;

    // controllers
    private NewSlideController slideController;
    private NewIntakeController intakeController;

    // rr
    private SampleTankDrive drive;

    @Override
    public void init() {
        // rr
        drive = new SampleTankDrive(hardwareMap);

        // runtimes
        primary = new GamepadRuntime(gamepad1, status, false);
        secondary = new GamepadRuntime(gamepad2, status, true);
        driverRuntime = new DriverRuntime(drive, gamepad1, status);

        // controllers
        slideController = new NewSlideController(hardwareMap);
        intakeController = new NewIntakeController(hardwareMap, slideController.getClaw(), slideController.getDistanceSensor());
    }

    @Override
    public void loop() {
        switch (status) {
            case paused:
                // just start runtimes
                if(primary.ready()) {
                    primary.start();
                }
                if(secondary.ready()) {
                    secondary.start();
                }

            case running:
                // just do thread checks
                intakeController.tick();

            case stopped:
                // call a function and then exit
        }
    }

    @Override
    public void stop(){
        slideController.killThreads();
        primary.stop();
        secondary.stop();
        driverRuntime.stop();
    }


}
