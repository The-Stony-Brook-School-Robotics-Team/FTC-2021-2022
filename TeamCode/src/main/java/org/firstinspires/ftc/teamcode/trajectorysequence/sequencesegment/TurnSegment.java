package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment;

import com.coyote.framework.core.geometry.Pose2d;
import com.coyote.framework.core.profile.MotionProfile;
import com.coyote.framework.core.trajectory.TrajectoryMarker;
import com.coyote.framework.core.util.Angle;

import java.util.List;

public final class TurnSegment extends SequenceSegment {
    private final double totalRotation;
    private final MotionProfile motionProfile;

    public TurnSegment(Pose2d startPose, double totalRotation, MotionProfile motionProfile, List<TrajectoryMarker> markers) {
        super(
                motionProfile.duration(),
                startPose,
                new Pose2d(
                        startPose.getX(), startPose.getY(),
                        Angle.norm(startPose.getHeading() + totalRotation)
                ),
                markers
        );

        this.totalRotation = totalRotation;
        this.motionProfile = motionProfile;
    }

    public final double getTotalRotation() {
        return this.totalRotation;
    }

    public final MotionProfile getMotionProfile() {
        return this.motionProfile;
    }
}
