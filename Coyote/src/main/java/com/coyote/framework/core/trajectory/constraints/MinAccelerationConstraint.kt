package com.coyote.framework.core.trajectory.constraints

import com.coyote.framework.core.geometry.Pose2d

/**
 * Composite constraint representing the minimum of its constituent acceleration constraints.
 */
class MinAccelerationConstraint(
    private val constraints: List<TrajectoryAccelerationConstraint>
) : TrajectoryAccelerationConstraint {
    override fun get(s: Double, pose: Pose2d, deriv: Pose2d, baseRobotVel: Pose2d) =
        constraints.map { it[s, pose, deriv, baseRobotVel] }.minOrNull()!!
}
