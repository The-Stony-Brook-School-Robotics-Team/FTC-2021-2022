package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Final;

import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.drive;
import static org.firstinspires.ftc.teamcode.common.teleop.OfficialTeleop.gamepad;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Color Tape Detection")
public class ColorReading extends LinearOpMode
{
    private final TrajectoryVelocityConstraint velocityConstraint = SampleMecanumDrive.getVelocityConstraint(30, 2, DriveConstants.TRACK_WIDTH);
    private final TrajectoryAccelerationConstraint accelerationConstraint = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL);

    public NormalizedColorSensor color;

    final int GAIN = 10;

    /** Array order is red, green, blue, alpha */
    final double[] RED = {.45, .13, .12, .62};
    final double[] BLUE = {.10, .36, .25, .66};
    final double[] WHITE = {.89, .86, .12, 1.00};



    @Override
    public void runOpMode() throws InterruptedException {


        color = hardwareMap.get(NormalizedColorSensor.class, "color");
        color.setGain(GAIN);
        waitForStart();

        while (opModeIsActive()) {
            if(color.getNormalizedColors().alpha > .8 && gamepad1.a){
                //drive.setPoseEstimate(new Pose2d(14, 65.5, 0));
                Pose2d currentPos = drive.getPoseEstimate();
                Pose2d target = new Pose2d(5.58, 64.47, -Math.toRadians(58));
                drive.followTrajectory(drive.trajectoryBuilder(currentPos)
                        .lineToSplineHeading(target, velocityConstraint, accelerationConstraint)
                        .build());
            }





        }
    }
}
