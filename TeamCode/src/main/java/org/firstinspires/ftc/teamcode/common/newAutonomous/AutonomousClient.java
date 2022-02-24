package org.firstinspires.ftc.teamcode.common.newAutonomous;

import static org.sbs.bears.robotframework.controllers.OpenCVController.doAnalysisMaster;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.Robot;
import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.controllers.DuckCarouselController;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideTarget;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;


public class AutonomousClient {
    final HardwareMap hardwareMap;
    final Telemetry telemetry;
    final AutonomousMode autonomousMode;
    Robot robot;

    RoadRunnerController roadRunnerController;
    SampleMecanumDrive roadRunnerDrive;

    OpenCVController openCVController;
    AutonomousSlideController slideController;
    IntakeControllerBlue intakeControllerBlue;
    IntakeControllerRed intakeControllerRed;
    DuckCarouselController duckCarouselController;
    Pose2d initialDropPosition = depositPositionAllianceBlueTOP;

    RevBlinkinLedDriver ledDriver;
    NormalizedColorSensor normalizedColorSensor;

    boolean objectIsInRobot;

    TowerHeightFromDuck firstDeliveryHeight;

    SlideTarget initialSlideTarget;

    public AutonomousClient(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode autonomousMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.autonomousMode = autonomousMode;

        initControllers(hardwareMap, telemetry, autonomousMode);

        objectIsInRobot = true;
        firstDeliveryHeight = TowerHeightFromDuck.NOT_YET_SET;

        initServoPositions();

        normalizedColorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        normalizedColorSensor.setGain(Configuration.colorSensorGain);
    }

    private void initControllers(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode) {
        this.robot = new Robot(hardwareMap, telemetry, mode);
        this.openCVController = robot.getCVctrl();
        this.roadRunnerController = robot.getRRctrl();
        this.roadRunnerDrive = roadRunnerController.getDrive();
        this.slideController = new AutonomousSlideController(hardwareMap, telemetry);
        this.intakeControllerBlue = robot.getIntakeCtrlBlue();
        this.intakeControllerRed = robot.getIntakeCtrlRed();
        this.duckCarouselController = robot.getDuckCtrl();
        this.ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "rgb");
    }

    private void initServoPositions() {
        intakeControllerBlue.setState(IntakeState.PARK);
        intakeControllerRed.setState(IntakeState.PARK);
        slideController.dumperServo.setPosition(SlideController.dumperPosition_CLOSED);
    }

    public void getInitialBlockDone() {
        readCamera();
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
        roadRunnerController.followLineToSpline(initialDropPosition);
        slideController.extendDropRetract_Autonomous(initialSlideTarget);
        objectIsInRobot = false;
    }

    public Thread getIntakeChecker() {
        return new Thread(() -> {  //Stop trajectory and load block into slide if robot has gotten the block.
            while (!objectIsInRobot) {
                if (Thread.interrupted())
                    return;

                objectIsInRobot = intakeControllerBlue.isObjectInPayload();
                Sleep.sleep(10);
            }
            intakeControllerBlue.setState(IntakeState.DUMP);
            roadRunnerController.endTrajectory();
        });
    }

    public void goPickUpBlock() {
        boolean isInWarehouse = false;
        objectIsInRobot = intakeControllerBlue.isObjectInPayload();
        while (!objectIsInRobot) {
            Thread intakeChecker = getIntakeChecker();

            //TODO:--------------------------------------------------------
            intakeChecker.start();
            ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

            if (isInWarehouse) {
                runTrajectoryGoPickUpBlockSecondary();
            } else {
                runTrajectoryGoPickUpBlock();
                isInWarehouse = true;
            }
            //Picking-up is not successful.
            intakeChecker.interrupt();
            //TODO:--------------------------------------------------------
        }
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void goDeliverBlock() {
        runTrajectoryGoDeliverBlock();

        slideController.extendDropRetract_Autonomous(SlideTarget.TOP_DEPOSIT);

        objectIsInRobot = false;
    }

    public void goParking() {
        intakeControllerBlue.setState(IntakeState.PARK);
        roadRunnerController.followLineToSpline(resetPositionB4WarehouseBlue);
        roadRunnerController.followLineToSpline(parkingPositionBlue);
    }

    public void readCamera() {
        doAnalysisMaster = true;
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        firstDeliveryHeight = openCVController.getWhichTowerHeight();
        openCVController.shutDown();
        switch (firstDeliveryHeight) {
            case ONE:
                initialSlideTarget = SlideTarget.BOTTOM_DEPOSIT;
                initialDropPosition = depositPositionAllianceBlueBOT;
                break;
            case TWO:
                initialSlideTarget = SlideTarget.MID_DEPOSIT;
                initialDropPosition = depositPositionAllianceBlueMID;
                break;
            case THREE:
                initialSlideTarget = SlideTarget.TOP_DEPOSIT;
                initialDropPosition = depositPositionAllianceBlueTOP;
                break;
        }
    }

    public void runTrajectoryGoPickUpBlock() {
        roadRunnerDrive.followTrajectory(
                roadRunnerDrive.trajectoryBuilder(
                        roadRunnerDrive.getPoseEstimate())
                        .lineToSplineHeading(A_FIX_HEADING_POSITION)
                        .addSpatialMarker(A_OPEN_PICK_UP_POSITION, () -> {
                            // This marker runs at the point that gets closest to the coordinate
                            intakeControllerBlue.setState(IntakeState.BASE);
                        })
                        .splineToLinearHeading(A_PICK_UP_BLOCK_POSITION, 0.0)
                        .build()
        );
    }

    public void runTrajectoryGoDeliverBlock() {
        roadRunnerDrive.followTrajectory(
                roadRunnerDrive.trajectoryBuilder(
                        roadRunnerDrive.getPoseEstimate(), true)
                        .splineToSplineHeading(B_FIX_HEADING_POSITION, Math.toRadians(175.0))
                        .splineToLinearHeading(B_PASS_PIPE_POSITION, -Math.toRadians(170.0))
                        .splineToSplineHeading(AutonomousClient.depositPositionAllianceBlueTOP, Math.toRadians(175.0))
                        .build()
        );
    }

    public void runTrajectoryGoPickUpBlockSecondary() {
        roadRunnerDrive.followTrajectory(
                roadRunnerDrive.trajectoryBuilder(
                        roadRunnerDrive.getPoseEstimate(), true)
                        .splineToConstantHeading(C_MOVE_AWAY_POSITION, -Math.toRadians(45.0))
                        .splineToSplineHeading(C_PICK_UP_BLOCK_POSITION, Math.toRadians(45.0))
                        .build()
        );
    }

    public static Pose2d startPositionBlue = new Pose2d(14, 65.5, 0);
    public static Pose2d resetPositionB4WarehouseBlue = new Pose2d(14, 75, 0);

    public static Pose2d A_PICK_UP_BLOCK_POSITION = new Pose2d(64.0, 66.5, 0.0);
    public static Pose2d A_FIX_HEADING_POSITION = new Pose2d(20.0, 67.0, 0.0);
    public static Vector2d A_OPEN_PICK_UP_POSITION = new Vector2d(28.5, 65.5);
    public static Pose2d B_FIX_HEADING_POSITION = new Pose2d(45.0, 66.0, 0.0);
    public static Pose2d B_PASS_PIPE_POSITION = new Pose2d(20.0, 68.0, 0.0);
    public static Vector2d C_MOVE_AWAY_POSITION = new Vector2d(45.0, 55.0);
    public static Pose2d C_PICK_UP_BLOCK_POSITION = new Pose2d(65.0, 66.5, Math.toRadians(20.0));

    public static Pose2d depositPositionAllianceBlueTOP = new Pose2d(7.58, 64.47, -Math.toRadians(55.0));
    public static Pose2d depositPositionAllianceBlueMID = new Pose2d(5.58, 64.47, -Math.toRadians(56.0));
    public static Pose2d depositPositionAllianceBlueBOT = new Pose2d(5.58, 64.47, -Math.toRadians(59.0));
    public static Pose2d parkingPositionBlue = new Pose2d(60, 75, 0);
}
