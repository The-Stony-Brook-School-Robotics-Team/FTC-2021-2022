package org.sbs.bears.coyote.servopkg;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Servo;

import org.jetbrains.annotations.NotNull;

public class ServoMotionFollower {

    // TODO: Add something
    public ServoMotionFollower() {
        super();
    }

    /**
     * Servo Motion Follower
     * @param servo
     * @param profile
     * @throws InterruptedException
     */
    public void asyncFollow(@NotNull Servo servo, @NotNull ServoMotionProfile profile) throws InterruptedException {
        double lastAng = servo.getPosition();
        long startTime = System.nanoTime();
        int i = 0;
        for(ServoMotionSegment segment : profile.getSegments()) {
            ServoMotionSegment nextSegment = profile.getSegment(i + 1);
            servo.setPosition(segment.getAngle());
            Thread.sleep(nextSegment.getDt() - segment.getDt());
            i++;
        }

    }





}
