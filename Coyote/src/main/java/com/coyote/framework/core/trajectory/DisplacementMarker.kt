package com.coyote.framework.core.trajectory

/**
 * Trajectory marker that is triggered when the specified displacement passes.
 */
data class DisplacementMarker(val producer: DisplacementProducer, val callback: MarkerCallback)
