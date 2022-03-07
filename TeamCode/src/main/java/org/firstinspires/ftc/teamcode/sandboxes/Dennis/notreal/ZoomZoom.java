package org.firstinspires.ftc.teamcode.sandboxes.Dennis.notreal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.controllers.DuckCarouselController;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.SlideTarget;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;

/**
 * This is not real, wake up
 */
@Autonomous(name = "Z - Zoom Zoom Mover (ONG!)")
public class ZoomZoom extends LinearOpMode {

    /**
     * ZoomZoom Interface Tag
     */
    private static String interfaceTag = "ZoomZoom";

    /**
     * The ZoomZoom States for The Robot
     */
    private enum ZoomZoomStates {
        IDLE,
        DEPOSIT,
        PREP,
        PICKUP,
        STUCK
    }

    /**
     * Current ZoomZoom State
     */
    private static ZoomZoomStates CurrentZoomZoomState = ZoomZoomStates.IDLE;

    /**
     * Controllers
     */
    public static OpenCVController openCVController;
    public static IntakeControllerBlue blueIntakeController;
    public static IntakeControllerRed redIntakeController;
    public static SlideController slideController;
    public static DuckCarouselController duckCarouselController;

    /**
     * RoadRunner Objects
     */
    public static SampleMecanumDrive drive;

    /**
     * Internal Management
     */
    public static Pose2d initialDepositPosition;
    public static TowerHeightFromDuck initialDepositHeight;
    public static SlideTarget initialSlideTarget;

    @Override
    public void runOpMode() {
        /**
         * Initialize RoadRunner
         */
        drive = new SampleMecanumDrive(hardwareMap);

        /**
         * Initialize Controllers
         */
        slideController = new SlideController(hardwareMap,telemetry);
        blueIntakeController = new IntakeControllerBlue(hardwareMap, slideController.blueDumperServo, telemetry);
       // redIntakeController = new IntakeControllerRed(hardwareMap, telemetry);
        duckCarouselController = new DuckCarouselController(hardwareMap, telemetry);
        openCVController = new OpenCVController(hardwareMap, telemetry, AutonomousMode.BlueStatesWarehouse);

        waitForStart();

        depositInitialBlock();



        /**
         * While a stop is not request && the timer is under 25 seconds
         */
        while(!isStopRequested() && NanoClock.system().seconds() < 25) {



        }
    }

    public static void depositInitialBlock() {
        readCamera();
        // Choose the deposit height
        switch (initialDepositHeight) {
            case ONE:
                initialSlideTarget = SlideTarget.BOTTOM_DEPOSIT;
                initialDepositPosition = ZoomZoomConfiguration.allianceDepositPosition_BOTTOM;
                break;
            case TWO:
                initialSlideTarget = SlideTarget.MID_DEPOSIT;
                initialDepositPosition = ZoomZoomConfiguration.allianceDepositPosition_MIDDLE;
                break;
            case THREE:
                initialSlideTarget = SlideTarget.TOP_DEPOSIT;
                initialDepositPosition = ZoomZoomConfiguration.allianceDepositPosition_TOP;
                break;
        }
        // Go to the deposit position
        drive.followTrajectory(
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(initialDepositPosition)
                        .build()
        );

        // Extend the slide
        slideController.extendDropRetract(initialSlideTarget);
    }

    public static void readCamera() {
        // Read from camera
        readCamera();
        initialDepositHeight = openCVController.getWhichTowerHeight();
        openCVController.shutDown();
    }

}
