package org.firstinspires.ftc.teamcode.common.newAutonomous;

import static org.sbs.bears.robotframework.controllers.OpenCVController.doAnalysisMaster;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.common.teleop.Configuration;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.Robot;
import org.sbs.bears.robotframework.controllers.DuckCarouselController;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideTarget;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;

@Config
public class AutonomousClientSafe {
    final boolean isTest = false;

    final HardwareMap hardwareMap;
    final Telemetry telemetry;
    final AutonomousMode autonomousMode;
    Robot robot;

    RoadRunnerController roadRunnerController;
    SampleMecanumDrive roadRunnerDrive;

    public static double startTime_s;

    OpenCVController openCVController;
    private boolean needToReadCamera = true;
    SlideController originalSlideController;
    IntakeControllerBlue intakeControllerBlue;
    IntakeControllerRed intakeControllerRed;
    DuckCarouselController duckCarouselController;
    Pose2d initialDropPosition = depositPositionBlueTOP;

    RevBlinkinLedDriver ledDriver;
    NormalizedColorSensor normalizedColorSensor;

    boolean objectIsInRobot;

    TowerHeightFromDuck firstDeliveryHeight;

    SlideTarget initialSlideTarget;

    Thread intakeChecker = new Thread();
    Thread extendRetractThread = new Thread();
    public static int AEarlyExtendSlideOffset = 17;
    public static int AEarlyRetractToTrajectoryOffset = 600;
    public static int AInitExtendMilliTimeOffset = 700;

    public AutonomousClientSafe(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode autonomousMode) {
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
        this.roadRunnerController.setPos(startPositionBlue);
        this.roadRunnerDrive = roadRunnerController.getDrive();
        this.originalSlideController = robot.getSlideCtrl();
        this.intakeControllerBlue = robot.getIntakeCtrlBlue();
        this.intakeControllerRed = robot.getIntakeCtrlRed();
        this.duckCarouselController = robot.getDuckCtrl();
        this.ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "rgb");
    }

    private void initServoPositions() {
        intakeControllerBlue.setState(IntakeState.PARK);
        intakeControllerRed.setState(IntakeState.PARK);
        originalSlideController.blueDumperServo.setPosition(SlideController.dumperPosition_CLOSED);
    }

    public void getInitialBlockDone() {
        if (needToReadCamera)
            readCamera();

        Thread initialSlideControlThread = new Thread(() -> {
            try {
                Thread.sleep(AInitExtendMilliTimeOffset);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            originalSlideController.extendDropRetract_NewAutonomous(initialSlideTarget);
        });

        initialSlideControlThread.start();

        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
        roadRunnerController.followLineToSpline(initialDropPosition);

        while (originalSlideController.slideMotor.getCurrentPosition() > AEarlyRetractToTrajectoryOffset) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        objectIsInRobot = false;
    }

    public Thread getIntakeChecker() {
        return new Thread(() -> {  //Stop trajectory and load block into slide if robot has gotten the block.
            objectIsInRobot = false;

            while (!objectIsInRobot) {
                if (Thread.currentThread().isInterrupted())
                    return;

                objectIsInRobot = intakeControllerBlue.isObjectInPayload();
//                Sleep.sleep(10);
            }

            //Block is in.
            roadRunnerController.endTrajectory();
            intakeControllerBlue.setState(IntakeState.DUMP);
        });
    }

    public void startTimer() {
        startTime_s = NanoClock.system().seconds();
    }

    public void pickUp() {
        if (!AutonomousTimer.canContinue())
            return;

        boolean isInWarehouse = false;
        objectIsInRobot = intakeControllerBlue.isObjectInPayload();
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        if (!intakeChecker.isAlive()) {
            intakeChecker = getIntakeChecker();
            intakeChecker.start();
        }

        while (!objectIsInRobot) {
            if (isInWarehouse) {
                runTrajectory_PickUpSecondary();
            } else {
                runTrajectory_PickUp();
                isInWarehouse = true;
            }

            if (!objectIsInRobot)    //PickUp is not successful.
                ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            if (!AutonomousTimer.canContinue(AutonomousTimer.CurrentState.PickUpSecondaryToDeposit))
                return;
        }

        if (intakeChecker.isAlive())
            intakeChecker.interrupt();

        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public static boolean TEST = true;

    public void deposit() {
        if (!AutonomousTimer.canContinue())
            return;

        if (!extendRetractThread.isAlive())
            extendRetractThread = startExtendDropRetractThread();
        else {
            extendRetractThread.interrupt();
            extendRetractThread = startExtendDropRetractThread();
        }

        runTrajectory_Deposit();

        while (originalSlideController.slideMotor.getCurrentPosition() > AEarlyRetractToTrajectoryOffset) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        objectIsInRobot = false;
    }

    private Thread startExtendDropRetractThread() {
        Thread extendDropRetractThread = new Thread(() -> {
            while (roadRunnerDrive.getPoseEstimate().getX() > AEarlyExtendSlideOffset) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            originalSlideController.extendDropRetract_NewAutonomous(SlideTarget.TOP_DEPOSIT);
            //TODO start path here
        });
        extendDropRetractThread.start();
        return extendDropRetractThread;
    }

    public void park() {
        runTrajectory_Park();
    }

    public void readCamera() {
        doAnalysisMaster = true;
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
        firstDeliveryHeight = openCVController.getWhichTowerHeight();
        openCVController.shutDown();
        switch (firstDeliveryHeight) {
            case ONE:
                initialSlideTarget = SlideTarget.BOTTOM_DEPOSIT;
                initialDropPosition = depositPositionBlueBOT;
                break;
            case TWO:
                initialSlideTarget = SlideTarget.MID_DEPOSIT;
                initialDropPosition = depositPositionBlueMID;
                break;
            default:
                initialSlideTarget = SlideTarget.TOP_DEPOSIT;
                initialDropPosition = depositPositionBlueTOP;
        }
        telemetry.addData("Camera Read: ", initialSlideTarget.toString());
        needToReadCamera = false;
    }

    public void runTrajectory_PickUp() {
        roadRunnerDrive.followTrajectory(
                roadRunnerDrive.trajectoryBuilder(
                        roadRunnerDrive.getPoseEstimate())
                        .lineToSplineHeading(PICK_UP_TRAJECTORY_FIX_HEADING_POSITION)
                        .splineToConstantHeading(PICK_UP_TRAJECTORY_PASS_PIPE_POSITION, PICK_UP_TRAJECTORY_PASS_PIPE_POSITION_TANGENT)
//                        .splineToConstantHeading(PICK_UP_TRAJECTORY_MOVE_OUT_POSITION, PICK_UP_TRAJECTORY_MOVE_OUT_POSITION_TANGENT)
                        .splineToConstantHeading(PICK_UP_TRAJECTORY_PICK_UP_POSITION, ZERO)
                        .addSpatialMarker(PICK_UP_TRAJECTORY_OPEN_PICK_UP_POSITION, () -> intakeControllerBlue.setState(IntakeState.BASE))
                        .addDisplacementMarker(this::runAntiBlockingChecker_PickUp)
                        .build()
        );
    }

    public void runTrajectory_Deposit() {
        roadRunnerDrive.followTrajectory(
                roadRunnerDrive.trajectoryBuilder(
                        roadRunnerDrive.getPoseEstimate(), true)
                        .lineToSplineHeading(DEPOSIT_TRAJECTORY_FIX_HEADING_POSITION)
                        .splineToLinearHeading(DEPOSIT_TRAJECTORY_PASS_PIPE_POSITION, Math.toRadians(-165.0))
                        .splineToSplineHeading(AutonomousClientBeta.firstDepositPositionBlueTOP, Math.toRadians(175.0))
                        .addDisplacementMarker(this::runAntiBlockingChecker_Deposit)
                        .build()
        );
    }

    public void runTrajectory_PickUpSecondary() {
        roadRunnerDrive.followTrajectorySequence(
                roadRunnerDrive.trajectorySequenceBuilder(roadRunnerDrive.getPoseEstimate())
                        .back(12.5)
                        .lineToLinearHeading(PICK_UP_SECONDARY_TRAJECTORY_PICK_UP_BLOCK_POSITION)
                        .build()
        );
    }

    public void runTrajectory_Park() {
        roadRunnerDrive.followTrajectory(
                roadRunnerDrive.trajectoryBuilder(roadRunnerDrive.getPoseEstimate())
                        .lineToSplineHeading(PICK_UP_TRAJECTORY_FIX_HEADING_POSITION)
                        .splineToLinearHeading(PARK_TRAJECTORY_PARK_POSITION, Math.toRadians(-10.0))
                        .addDisplacementMarker(this::runAntiBlockingChecker_Park)
                        .build()
        );
    }

    private void runAntiBlockingChecker_PickUp() {
        if (roadRunnerController.getPos().getX() < ANTI_BLOCKING_CHECKER_PICK_UP_X) {
            stopRoadRunner();
            roadRunnerDrive.followTrajectory(
                    roadRunnerDrive.trajectoryBuilder(roadRunnerDrive.getPoseEstimate(), true)
                            .lineToLinearHeading(ABC_RESET_POSITION_PICK_UP)
                            .build()
            );
            pickUp();
        }
    }

    private void runAntiBlockingChecker_Deposit() {
        if (roadRunnerController.getPos().getX() > ANTI_BLOCKING_CHECKER_DEPOSIT_X) {
            stopRoadRunner();
            roadRunnerDrive.followTrajectory(
                    roadRunnerDrive.trajectoryBuilder(roadRunnerDrive.getPoseEstimate(), true)
                            .lineToLinearHeading(ABC_RESET_POSITION_DEPOSIT)
                            .build()
            );
            deposit();
        }
    }

    private void runAntiBlockingChecker_Park() {
        if (roadRunnerController.getPos().getX() < ANTI_BLOCKING_CHECKER_PARK_X) {
            stopRoadRunner();
            roadRunnerDrive.followTrajectory(
                    roadRunnerDrive.trajectoryBuilder(roadRunnerDrive.getPoseEstimate(), true)
                            .lineToLinearHeading(ABC_RESET_POSITION_PARK)
                            .build()
            );
            park();
        }
    }

    public void stopRoadRunner() {
        roadRunnerController.endTrajectory();
    }

    private static final int ZERO = 0;
    private static final double TANGENT = 30.0;

    public static Pose2d startPositionBlue = new Pose2d(14.0, 65.5, ZERO);
    public static Pose2d WHITE_LINE_POSITION = new Pose2d(29.5, 65.5, ZERO);
    private static final double ANTI_BLOCKING_CHECKER_DEPOSIT_X = 25.0;
    private static final double ANTI_BLOCKING_CHECKER_PICK_UP_X = 40.0;
    private static final double ANTI_BLOCKING_CHECKER_PARK_X = 32.0;

    private static final Vector2d PICK_UP_TRAJECTORY_OPEN_PICK_UP_POSITION = new Vector2d(20.0, 65.5);
    private static final Pose2d PICK_UP_TRAJECTORY_FIX_HEADING_POSITION = new Pose2d(14.0, 65.5, ZERO);
    private static final Vector2d PICK_UP_TRAJECTORY_PASS_PIPE_POSITION = new Vector2d(35.0, 66.0);
    private static final double PICK_UP_TRAJECTORY_PASS_PIPE_POSITION_TANGENT = Math.toRadians(-15.0);
    private static final Vector2d PICK_UP_TRAJECTORY_MOVE_OUT_POSITION = new Vector2d(45.0, 64.0);
    private static final double PICK_UP_TRAJECTORY_MOVE_OUT_POSITION_TANGENT = Math.toRadians(-10.0);
    private static final Vector2d PICK_UP_TRAJECTORY_PICK_UP_POSITION = new Vector2d(56.0, 65.5);

    private static final Pose2d DEPOSIT_TRAJECTORY_FIX_HEADING_POSITION = new Pose2d(40.0, 67.0, ZERO);
    private static final Pose2d DEPOSIT_TRAJECTORY_PASS_PIPE_POSITION = new Pose2d(20.0, 65.5, ZERO);   //Heading is identical to B_FIX_HEADING_POSITION
    private static final double DEPOSIT_TRAJECTORY_START_SLIDE_X = 20;

    private static final Pose2d PICK_UP_SECONDARY_TRAJECTORY_PICK_UP_BLOCK_POSITION = new Pose2d(57.0, 65.0, Math.toRadians(0.0));

    private static final Pose2d PARK_TRAJECTORY_PARK_POSITION = new Pose2d(50.0, 66.0, 0);

    private static final Pose2d ABC_RESET_POSITION_PICK_UP = new Pose2d(10.0, 64.0, 0);
    private static final Pose2d ABC_RESET_POSITION_DEPOSIT = new Pose2d(55.0, 64.0, 0.0);
    private static final Pose2d ABC_RESET_POSITION_PARK = new Pose2d(10.0, 64.0, 0);
    private static final Vector2d ABC_CHECK_POSITION_PICK_UP = new Vector2d(50.5, 65.5);
    private static final Vector2d ABC_CHECK_POSITION_DEPOSIT = new Vector2d(12.0, 65.5);
    private static final Vector2d ABC_CHECK_POSITION_PARK = ABC_CHECK_POSITION_PICK_UP;


    public static Pose2d depositPositionBlueTOP = new Pose2d(7.15, 63.0, -Math.toRadians(33.0));
    public static Pose2d depositPositionBlueMID = new Pose2d(7.15, 63.0, -Math.toRadians(33.0));
    public static Pose2d depositPositionBlueBOT = new Pose2d(7.15, 63.0, -Math.toRadians(33.0));
}
