package com.coyote.framework.core.localization

import com.coyote.framework.core.geometry.Pose2d

/**
 * Generic interface for estimating robot pose over time.
 */
interface Localizer {

    /**
     * Current robot pose estimate.
     */
    var poseEstimate: Pose2d

    /**
     * Current robot pose velocity (optional)
     */
    val poseVelocity: Pose2d?

    /**
     * Completes a single localization update.
     */
    fun update()
}
