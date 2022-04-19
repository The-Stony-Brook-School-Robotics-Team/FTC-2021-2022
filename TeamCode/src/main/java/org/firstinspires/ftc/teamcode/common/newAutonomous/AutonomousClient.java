package org.firstinspires.ftc.teamcode.common.newAutonomous;

import static org.sbs.bears.robotframework.controllers.OpenCVController.doAnalysisMaster;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.firstinspires.ftc.teamcode.common.official.teleop.v1.Configuration;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.controllers.DuckCarouselController;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.OpenCVController;
import org.sbs.bears.robotframework.controllers.RoadRunnerController;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideTarget;
import org.sbs.bears.robotframework.enums.TowerHeightFromDuck;

//@Config
public class AutonomousClient {

    final HardwareMap hardwareMap;
    final Telemetry telemetry;
    final AutonomousMode autonomousMode;

    RoadRunnerController roadRunnerController;
    SampleMecanumDrive roadRunnerDrive;

    OpenCVController openCVController;
    private boolean needToReadCamera = true;
    SlideController slideController;
    IntakeControllerBlue intakeControllerBlue;
    IntakeControllerRed intakeControllerRed;
    DuckCarouselController duckCarouselController;
    Pose2d initialDropPosition = depositPositionBlueTOP;

    RevBlinkinLedDriver ledDriver;
    NormalizedColorSensor normalizedColorSensor;

    boolean objectIsInRobot = true;

    TowerHeightFromDuck firstDeliveryHeight;

    SlideTarget initialSlideTarget;

    Thread intakeChecker = new Thread();
    Thread extendRetractThread = new Thread();
    public volatile boolean needToStopAllThreads = false;
    public static int EARLY_TRAJECTORY_TO_EXTEND_X = 10;
    public static int EARLY_RETRACT_TO_TRAJECTORY_SLIDE_POSE = 600;
    public static int INITIAL_TRAJECTORY_TO_EXTEND_TIME_OFFSET = 700;

    public static double PICK_UP_POSE_X = 55.0;
    public static double PICK_UP_POSE_Y = 66.0;
    public static double DEPOSIT_POSE_X = 7.15;
    public static double DEPOSIT_POSE_Y = 61.0;
    public static double PICK_UP_FIX_HEADING_X = 14.0;
    public static double PICK_UP_FIX_HEADING_Y = 66.5;
    public static double DEPOSIT_FIX_HEADING_X = 40.0;
    public static double DEPOSIT_FIX_HEADING_Y = 67.0;
    public static double DEPOSIT_PASS_PIPE_X = 18.0;
    public static double DEPOSIT_PASS_PIPE_Y = 66.0;

    public AutonomousClient(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode autonomousMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.autonomousMode = autonomousMode;

        initControllers(hardwareMap, telemetry, autonomousMode);

        initServoPositions();

        normalizedColorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        normalizedColorSensor.setGain(Configuration.colorSensorGain);
    }

    private void initControllers(HardwareMap hardwareMap, Telemetry telemetry, AutonomousMode mode) {
        this.openCVController = new OpenCVController(hardwareMap, telemetry, mode);

        this.roadRunnerController = new RoadRunnerController(hardwareMap, telemetry);
        this.roadRunnerController.setPos(startPositionBlue);
        this.roadRunnerDrive = roadRunnerController.getDrive();

        this.slideController = new SlideController(hardwareMap, telemetry);
        this.intakeControllerBlue = new IntakeControllerBlue(hardwareMap, slideController.blueDumperServo, telemetry);
        this.intakeControllerRed = new IntakeControllerRed(hardwareMap, slideController.redDumperServo, telemetry);
        this.duckCarouselController = new DuckCarouselController(hardwareMap, telemetry);
        this.ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "rgb");
    }

    private void initServoPositions() {
        intakeControllerBlue.setState(IntakeState.PARK);
        intakeControllerRed.setState(IntakeState.PARK);
        slideController.blueDumperServo.setPosition(SlideController.dumperPosition_CLOSED);
    }

    public void getInitialBlockDone() {
        if (needToReadCamera)
            readCamera();

        Thread initialSlideControlThread = new Thread(() -> {
            try {
                Thread.sleep(INITIAL_TRAJECTORY_TO_EXTEND_TIME_OFFSET);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (!needToStopAllThreads)
                slideController.extendDropRetract(initialSlideTarget);
        });

        initialSlideControlThread.start();

        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE);
        roadRunnerDrive.followTrajectory(
                roadRunnerDrive.trajectoryBuilder(
                        roadRunnerDrive.getPoseEstimate())
                        .lineToLinearHeading(initialDropPosition)
                        .build()
        );

        while (slideController.slideMotor.getCurrentPosition() > EARLY_RETRACT_TO_TRAJECTORY_SLIDE_POSE) {
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
            stopRoadRunner();
            intakeControllerBlue.setState(IntakeState.DUMP);
        });
    }

    public void pickUp() {
        if (!AutonomousTimer.canContinue(AutonomousTimer.CurrentState.DepositToPickUp))
            return;

        boolean isInWarehouse = false;
        objectIsInRobot = intakeControllerBlue.isObjectInPayload();
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        if (intakeChecker.isAlive())
            intakeChecker.interrupt();

        intakeChecker = getIntakeChecker();
        intakeChecker.start();

        while (!objectIsInRobot) {
            if (isInWarehouse) {
                runTrajectory_PickUpSecondary();
            } else {
                runTrajectory_PickUp();
                isInWarehouse = true;
            }

            if (!objectIsInRobot)    //PickUp is not successful.
                ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            if (!AutonomousTimer.canContinue(AutonomousTimer.CurrentState.PickUpToDeposit))
                return;
        }

        if (intakeChecker.isAlive())
            intakeChecker.interrupt();

        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public static boolean TEST = true;

    public void deposit() {
        if (!AutonomousTimer.canContinue(AutonomousTimer.CurrentState.PickUpToDeposit))
            return;

        if (extendRetractThread.isAlive())
            extendRetractThread.interrupt();

        extendRetractThread = startExtendDropRetractThread();

        runTrajectory_Deposit();

        while (slideController.slideMotor.getCurrentPosition() > EARLY_RETRACT_TO_TRAJECTORY_SLIDE_POSE) {
            if (needToStopAllThreads)
                return;

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
            while (roadRunnerDrive.getPoseEstimate().getX() > EARLY_TRAJECTORY_TO_EXTEND_X) {
                if (needToStopAllThreads)
                    return;

                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            slideController.extendDropRetract(SlideTarget.TOP_DEPOSIT);
            //TODO start path here
        });
        extendDropRetractThread.start();
        return extendDropRetractThread;
    }

    public void park() {
        if (AutonomousTimer.currentState == AutonomousTimer.CurrentState.PickUpToDeposit) {  //Already inside warehouse
            roadRunnerDrive.followTrajectory(
                    roadRunnerDrive.trajectoryBuilder(
                            roadRunnerDrive.getPoseEstimate())
                            .lineToLinearHeading(AutonomousClient.PARK_TRAJECTORY_PARK_POSITION)
                            .build()
            );
        } else
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
        telemetry.update();
        needToReadCamera = false;
    }

    public void runTrajectory_PickUp() {
        roadRunnerDrive.followTrajectory(
                roadRunnerDrive.trajectoryBuilder(
                        roadRunnerDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(PICK_UP_FIX_HEADING_X, PICK_UP_FIX_HEADING_Y, 0.0))
                        .splineToConstantHeading(new Vector2d(PICK_UP_POSE_X, PICK_UP_POSE_Y), 0.0)
                        .addSpatialMarker(new Vector2d(40.0, 70.0), () -> roadRunnerDrive.setPoseEstimate(new Pose2d(roadRunnerDrive.getPoseEstimate().getX(), 65.5, roadRunnerDrive.getPoseEstimate().getHeading())))
                        .addSpatialMarker(PICK_UP_TRAJECTORY_OPEN_PICK_UP_POSITION, () -> intakeControllerBlue.setState(IntakeState.BASE))
                        .addDisplacementMarker(this::runAntiBlockingChecker_PickUp)
                        .build()
        );
    }

    public void runRawPickUpTrajectory() {
        roadRunnerDrive.followTrajectory(
                roadRunnerDrive.trajectoryBuilder(
                        roadRunnerDrive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(PICK_UP_FIX_HEADING_X, PICK_UP_FIX_HEADING_Y, 0.0))
                        .splineToConstantHeading(new Vector2d(PICK_UP_POSE_X, PICK_UP_POSE_Y), 0.0)
                        .build()
        );
    }

    public void runTrajectory_Deposit() {
        roadRunnerDrive.followTrajectory(
                roadRunnerDrive.trajectoryBuilder(
                        roadRunnerDrive.getPoseEstimate(), true)
                        .lineToSplineHeading(new Pose2d(DEPOSIT_FIX_HEADING_X, DEPOSIT_FIX_HEADING_Y, 0.0))
                        .splineToConstantHeading(new Vector2d(DEPOSIT_PASS_PIPE_X, DEPOSIT_PASS_PIPE_Y), Math.toRadians(-160.0))
                        .splineToSplineHeading(new Pose2d(DEPOSIT_POSE_X, DEPOSIT_POSE_Y, Math.toRadians(-33.0)), Math.toRadians(-175.0))
                        .addSpatialMarker(new Vector2d(35.0, 70.0), () -> roadRunnerDrive.setPoseEstimate(new Pose2d(roadRunnerDrive.getPoseEstimate().getX(), 65.5, roadRunnerDrive.getPoseEstimate().getHeading())))
                        .addDisplacementMarker(this::runAntiBlockingChecker_Deposit)
                        .build()
        );
    }

    public void runRawDepositTrajectory() {
        roadRunnerDrive.followTrajectory(
                roadRunnerDrive.trajectoryBuilder(
                        roadRunnerDrive.getPoseEstimate(), true)
                        .lineToSplineHeading(new Pose2d(DEPOSIT_FIX_HEADING_X, DEPOSIT_FIX_HEADING_Y, 0.0))
                        .splineToConstantHeading(new Vector2d(DEPOSIT_PASS_PIPE_X, DEPOSIT_PASS_PIPE_Y), Math.toRadians(-160.0))
                        .splineToSplineHeading(new Pose2d(DEPOSIT_POSE_X, DEPOSIT_POSE_Y, Math.toRadians(-33.0)), Math.toRadians(-175.0))
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
            intakeControllerBlue.setState(IntakeState.PARK);
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
            extendRetractThread.interrupt();

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

    public static Pose2d startPositionBlue = new Pose2d(14.0, 65.5, ZERO);
    public static Pose2d WHITE_LINE_POSITION = new Pose2d(29.5, 65.5, ZERO);
    private static final double ANTI_BLOCKING_CHECKER_DEPOSIT_X = 25.0;
    private static final double ANTI_BLOCKING_CHECKER_PICK_UP_X = 40.0;
    private static final double ANTI_BLOCKING_CHECKER_PARK_X = 32.0;

    private static final Vector2d PICK_UP_TRAJECTORY_OPEN_PICK_UP_POSITION = new Vector2d(17.0, 66);
    private static final Pose2d PICK_UP_TRAJECTORY_FIX_HEADING_POSITION = new Pose2d(10.0, 65.5, ZERO);

    private static final Pose2d PICK_UP_SECONDARY_TRAJECTORY_PICK_UP_BLOCK_POSITION = new Pose2d(56.0, 64.0, Math.toRadians(0.0));

    public static final Pose2d PARK_TRAJECTORY_PARK_POSITION = new Pose2d(50.0, 66.0, 0);

    private static final Pose2d ABC_RESET_POSITION_PICK_UP = new Pose2d(10.0, 64.0, 0);
    private static final Pose2d ABC_RESET_POSITION_DEPOSIT = new Pose2d(55.0, 64.0, 0.0);
    private static final Pose2d ABC_RESET_POSITION_PARK = new Pose2d(10.0, 64.0, 0);

    public static Pose2d depositPositionBlueTOP = new Pose2d(2.0, 63.0, -Math.toRadians(33.0));
    public static Pose2d depositPositionBlueMID = new Pose2d(2.0, 63.0, -Math.toRadians(34.0));
    public static Pose2d depositPositionBlueBOT = new Pose2d(2.0, 63.0, -Math.toRadians(35.0));
}
