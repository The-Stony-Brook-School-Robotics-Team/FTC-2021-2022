package org.sbs.bears.Tank;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.sbs.bears.robotframework.enums.IntakeState;

public class twoinchNoRR {
        private DriveController drive;
        private NewSlideController slideController;
        private NewRedIntakeController intakeController;
        private Double startTime;
        private Double elapsedtime;
        private Double curveMultiplier = 0.95;
        private Double curveY = 58.3;



        public twoinchNoRR(HardwareMap hardwareMap){
            drive = new DriveController(hardwareMap);
            slideController = new NewSlideController(hardwareMap);
            intakeController = new NewRedIntakeController(hardwareMap, slideController.getClaw(),slideController.getDistanceSensor() );

        }
        public void initialDeposit(){
            drive.driveTo((int) DriveController.inchesToEncoderTicks(-22));
            while(drive.rb.isBusy()){

            }
            slideController.extendDropRetract(SlideConstants.slideMotorPosition_THREE_CLOSE, SlideConstants.flipper_THREE_CLOSE, SlideConstants.potentiometer_AUTON);
            drive.driveTo((int)DriveController.inchesToEncoderTicks(-9.3));

        }

        public void intakeCycle(){
            elapsedtime = NanoClock.system().seconds() - startTime;
            while(elapsedtime < 2.5) { //TODO time
                intakeController.setState(IntakeState.DUMP);
                while (!intakeController.isFreight() && drive.rb.isBusy()) {
                    drive.update();
                    intakeController.tick();
                }
                if(!intakeController.isFreight()){
                    while(!intakeController.isFreight() && drive.rb.isBusy()){
                        drive.update();
                        intakeController.tick();
                    }
                }

                slideController.dropFreight();
                slideController.retract();
                drive.driveTo((int)DriveController.inchesToEncoderTicks(35));
            }
        }


    static Pose2d blueStartPose = new Pose2d(14, 65.5, 0);
 /**
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

    Trajectory goIntakeFromDeposit; **/
}
