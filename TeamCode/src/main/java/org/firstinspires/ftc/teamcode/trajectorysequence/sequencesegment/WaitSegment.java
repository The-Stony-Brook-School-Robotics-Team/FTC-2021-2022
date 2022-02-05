package org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment;

import com.coyote.framework.core.geometry.Pose2d;
import com.coyote.framework.core.trajectory.TrajectoryMarker;

import java.util.List;

public final class WaitSegment extends SequenceSegment {
    public WaitSegment(Pose2d pose, double seconds, List<TrajectoryMarker> markers) {
        super(seconds, pose, pose, markers);
    }
}
