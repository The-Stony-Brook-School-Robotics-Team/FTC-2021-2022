package org.firstinspires.ftc.teamcode.sandboxes.Dennis.notreal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.SlideController;

/**
 * This is not real, wake up
 */
@Autonomous(name = "Z - Zoom Zoom Mover (ONG!)")
public class ZoomZoom extends OpMode {

    private enum ZoomZoomStates {
        IDLE,
        DEPOSIT,
        PREP,
        PICKUP
    }

    private ZoomZoomStates CurrentZoomZoomState = ZoomZoomStates.IDLE;

    /**
     * Controllers
     */
    private IntakeControllerBlue blueIntakeController;
    private IntakeControllerRed redIntakeController;
    private SlideController slideController;

    /**
     * RoadRunner Objects
     */
    private SampleMecanumDrive drive;

    @Override
    public void init() {
        /**
         * Initialize Controllers
         */
        // blueIntakeController = new IntakeControllerBlue(this.hardwareMap);

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

    }

}
