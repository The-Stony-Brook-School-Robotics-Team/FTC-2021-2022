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
    Pose2d initialDropPosition = firstDepositPositionBlueTOP;

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
        }
        AntiBlockingChecker_PickUp();
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void goDepositBlock() {
        runTrajectoryGoDepositBlock();
        slideController.extendDropRetract_Autonomous(SlideTarget.TOP_DEPOSIT);
        objectIsInRobot = false;
        AntiBlockingChecker_Deposit();
    }

    public void goParking() {
        runTrajectoryGoParking();
        AntiBlockingChecker_Parking();
    }

    public void readCamera() {
        doAnalysisMaster = true;
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        firstDeliveryHeight = openCVController.getWhichTowerHeight();
        openCVController.shutDown();
        switch (firstDeliveryHeight) {
            case ONE:
                initialSlideTarget = SlideTarget.BOTTOM_DEPOSIT;
                initialDropPosition = firstDepositPositionBlueBOT;
                break;
            case TWO:
                initialSlideTarget = SlideTarget.MID_DEPOSIT;
                initialDropPosition = firstDepositPositionBlueMID;
                break;
            case THREE:
                initialSlideTarget = SlideTarget.TOP_DEPOSIT;
                initialDropPosition = firstDepositPositionBlueTOP;
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

    public void runTrajectoryGoDepositBlock() {
        roadRunnerDrive.followTrajectory(
                roadRunnerDrive.trajectoryBuilder(
                        roadRunnerDrive.getPoseEstimate(), true)
                        .splineToSplineHeading(B_FIX_HEADING_POSITION, Math.toRadians(175.0))
                        .splineToLinearHeading(B_PASS_PIPE_POSITION, -Math.toRadians(170.0))
                        .splineToSplineHeading(AutonomousClient.firstDepositPositionBlueTOP, Math.toRadians(175.0))
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

    public void runTrajectoryGoParking() {
        roadRunnerDrive.followTrajectory(
                roadRunnerDrive.trajectoryBuilder(roadRunnerDrive.getPoseEstimate())
                        .lineToSplineHeading(A_FIX_HEADING_POSITION)
                        .splineToLinearHeading(parkingPositionBlue, 0.0)
                        .build()
        );
    }

    public void AntiBlockingChecker_Deposit() {
        if (roadRunnerController.getPos().getX() > ANTI_BLOCKING_CHECKER_DEPOSIT_X) {
            roadRunnerDrive.followTrajectory(
                    roadRunnerDrive.trajectoryBuilder(roadRunnerDrive.getPoseEstimate(), true)
                            .lineToLinearHeading(ABC_DEPOSIT_POSITION)
                            .build()
            );
            goDepositBlock();
        }
    }

    public void AntiBlockingChecker_PickUp() {
        if (roadRunnerController.getPos().getX() < ANTI_BLOCKING_CHECKER_PICK_UP_X) {
            roadRunnerDrive.followTrajectory(
                    roadRunnerDrive.trajectoryBuilder(roadRunnerDrive.getPoseEstimate(), true)
                            .lineToLinearHeading(ABC_PICK_UP_POSITION)
                            .build()
            );
            goPickUpBlock();
        }
    }

    public void AntiBlockingChecker_Parking() {
        if (roadRunnerController.getPos().getX() < ANTI_BLOCKING_CHECKER_PARKING_X) {
            roadRunnerDrive.followTrajectory(
                    roadRunnerDrive.trajectoryBuilder(roadRunnerDrive.getPoseEstimate(), true)
                            .lineToLinearHeading(ABC_PARKING_POSITION)
                            .build()
            );
            goParking();
        }
    }

    public final static int ZERO = 0;

    public static Pose2d startPositionBlue = new Pose2d(14.0, 65.5, ZERO);
    public static Pose2d WHITE_LINE_POSITION = new Pose2d(29.5, 65.5, ZERO);
    public static double ANTI_BLOCKING_CHECKER_DEPOSIT_X = 25.0;
    public static double ANTI_BLOCKING_CHECKER_PICK_UP_X = 32.0;
    public static double ANTI_BLOCKING_CHECKER_PARKING_X = 32.0;

    public static Vector2d A_OPEN_PICK_UP_POSITION = new Vector2d(28.5, 65.5);
    public static Pose2d A_FIX_HEADING_POSITION = new Pose2d(20.0, 68.0, ZERO);
    public static Pose2d A_PICK_UP_BLOCK_POSITION = new Pose2d(64.0, 66.5, ZERO);   //Heading is identical to A_FIX_HEADING_POSITION
    public static Pose2d B_FIX_HEADING_POSITION = new Pose2d(45.0, 66.0, ZERO);
    public static Pose2d B_PASS_PIPE_POSITION = new Pose2d(20.0, 68.0, ZERO);   //Heading is identical to B_FIX_HEADING_POSITION
    public static Vector2d C_MOVE_AWAY_POSITION = new Vector2d(45.0, 55.0);
    public static Pose2d C_PICK_UP_BLOCK_POSITION = new Pose2d(64.0, 66.0, Math.toRadians(20.0));

    public static Pose2d ABC_DEPOSIT_POSITION = new Pose2d(45.0, 66.0, 0.0);
    public static Pose2d ABC_PICK_UP_POSITION = new Pose2d(14.0, 65.5, 0);
    public static Pose2d ABC_PARKING_POSITION = new Pose2d(14.0, 65.5, 0);

    public static Pose2d firstDepositPositionBlueTOP = new Pose2d(7.58, 64.47, -Math.toRadians(30.0));
    public static Pose2d firstDepositPositionBlueMID = new Pose2d(5.58, 64.47, -Math.toRadians(31.0));
    public static Pose2d firstDepositPositionBlueBOT = new Pose2d(5.58, 64.47, -Math.toRadians(34.0));
    public static Pose2d parkingPositionBlue = new Pose2d(60, 75, 0);
}
