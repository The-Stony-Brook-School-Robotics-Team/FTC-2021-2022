package org.sbs.bears.coyote.servopkg;

import com.qualcomm.robotcore.hardware.Servo;

import org.jetbrains.annotations.NotNull;

public class ServoMotionFollower {

    // TODO: Add something
    public ServoMotionFollower() {
        return;
    }

    public void follow(@NotNull Servo servo, @NotNull ServoMotionProfile profile) {
        double lastAng = servo.getPosition();
        for(ServoMotionSegment segment : profile.getSegments()) {
            segment.
        }
    }





}
