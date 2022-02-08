package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Color Tape Detection")
public class ColorReading extends LinearOpMode
{
    private final TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(30, 2, DriveConstants.TRACK_WIDTH);
    private final TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

    public NormalizedColorSensor color;
    private SampleMecanumDrive drive;

    final int GAIN = 10;

    /** Array order is red, green, blue, alpha */




    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        color = hardwareMap.get(NormalizedColorSensor.class, "color");
        color.setGain(GAIN);
        waitForStart();

        while (opModeIsActive()) {
            if(color.getNormalizedColors().alpha > .8 && gamepad1.a){

                drive.setPoseEstimate(new Pose2d(32, 65.5, 0));
                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(14, 65.5), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(5.58, 64.47, -Math.toRadians(58)), Math.toRadians(0), velocityConstraint, accelerationConstraint)
                        .build());

                Thread.sleep(2000);


                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToSplineHeading(new Pose2d(14,80,0), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(32, 80), Math.toRadians(0))
                        .build());

            }





        }
    }
}
