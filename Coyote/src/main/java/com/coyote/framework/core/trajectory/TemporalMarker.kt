package com.coyote.framework.core.trajectory

/**
 * Trajectory marker that is triggered when the specified time passes.
 */
data class TemporalMarker(val producer: TimeProducer, val callback: MarkerCallback)
