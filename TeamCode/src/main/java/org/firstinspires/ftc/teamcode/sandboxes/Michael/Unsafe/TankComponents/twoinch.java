package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.sbs.bears.robotframework.enums.IntakeState;

public class twoinch  {
        private SampleTankDrive drive;
        private NewSlideController slideController;
        private NewRedIntakeController intakeController;
        private Double startTime;
        private Double elapsedtime;
        private Double curveMultiplier = 0.95;
        private Double curveY = 58.3;



        public twoinch(HardwareMap hardwareMap){
            drive = new SampleTankDrive(hardwareMap);
            slideController = new NewSlideController(hardwareMap);
            intakeController = new NewRedIntakeController(hardwareMap, slideController.getClaw(),slideController.getDistanceSensor(), slideController.getSlideMotor());

        }

        public void initialDeposit(){
            drive.followTrajectory(goInitialDeposit);
            slideController.extendDropRetract(SlideConstants.slideMotorPosition_THREE_CLOSE, SlideConstants.flipper_THREE_CLOSE, SlideConstants.potentiometer_AUTON);
            drive.followTrajectoryAsync(goIntakeFromInitialTrajectory);

        }

        public void intakeCycle(){
            elapsedtime = NanoClock.system().seconds() - startTime;
            while(elapsedtime < 2.5) { //TODO time
                curveY *= curveMultiplier;
                intakeController.setState(IntakeState.DUMP);
                while (!intakeController.isFreight() && drive.isBusy()) {
                    drive.update();
                    intakeController.tick();
                }
                drive.breakFollowing();
                if(!intakeController.isFreight()){
                    drive.followTrajectoryAsync(goCurveTowardsFreight);
                    while(!intakeController.isFreight() && drive.isBusy()){
                        drive.update();
                        intakeController.tick();
                    }
                }
                drive.followTrajectory(goDepositFromIntake);
                slideController.dropFreight();
                slideController.retract();
                //drive.followTrajectory();
            }
        }


    static Pose2d blueStartPose = new Pose2d(14, 65.5, 0);
    Trajectory goInitialDeposit = new TrajectoryBuilder(blueStartPose, SampleTankDrive.VEL_CONSTRAINT, SampleTankDrive.accelConstraint)

            .lineToConstantHeading(new Vector2d(-4.7, 65.5))
            .build();
    Trajectory goIntakeFromInitialTrajectory = new TrajectoryBuilder(drive.getPoseEstimate(), SampleTankDrive.VEL_CONSTRAINT, SampleTankDrive.accelConstraint)
           .lineToConstantHeading(new Vector2d(36,65.5)) //to end of intake area?
            .build();

    Trajectory goCurveTowardsFreight = new TrajectoryBuilder(drive.getPoseEstimate(), SampleTankDrive.VEL_CONSTRAINT, SampleTankDrive.accelConstraint)
            .splineTo(new Vector2d(56.5, curveY), Math.toRadians(0))
            .build();

    Trajectory goDepositFromIntake = new TrajectoryBuilder(drive.getPoseEstimate(), SampleTankDrive.VEL_CONSTRAINT, SampleTankDrive.accelConstraint)
            .addSpatialMarker(new Vector2d(30, 75), () -> {
                slideController.extend(SlideConstants.slideMotorPosition_THREE_CLOSE, SlideConstants.flipper_THREE_CLOSE, SlideConstants.potentiometer_THREE_DEPOSIT);
            })
            .splineTo(new Vector2d(35, 75), Math.toRadians(0))
            .build();

    Trajectory goIntakeFromDeposit;
}
