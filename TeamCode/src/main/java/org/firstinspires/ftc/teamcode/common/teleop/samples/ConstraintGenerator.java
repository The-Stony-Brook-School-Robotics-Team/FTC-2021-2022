package org.firstinspires.ftc.teamcode.common.teleop.samples;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.DriveConstantsMain;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * Makes Constraints Based Off Of Given Values
 */
public class ConstraintGenerator {

    /**
     * Returns a new velocity constraint
     * @param velocity velocity to constrain to
     * @return velocity constraint
     */
    public TrajectoryVelocityConstraint getVelocityConstraint(int velocity) {
        return SampleMecanumDrive.getVelocityConstraint(velocity, DriveConstantsMain.MAX_ANG_VEL, DriveConstantsMain.TRACK_WIDTH);
    }

    /**
     * Returns a new acceleration constraint
     * @param acceleration acceleration to constrain to
     * @return acceleration constraint
     */
    public TrajectoryAccelerationConstraint getAccelerationConstraint(double acceleration) {
        return SampleMecanumDrive.getAccelerationConstraint(acceleration);
    }

}
