package org.sbs.bears.coyote.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.jetbrains.annotations.NotNull;

public class ServoMotionFollower {

    // TODO: Add something
    public ServoMotionFollower() {
        super();
    }

    /**
     * Servo Motion Follower
     * @param servo the servo that you want the follower to follow
     * @param profile the motion profile to follow
     * @throws InterruptedException because the thread that this sleeps can be stopped
     */
    public void asyncFollow(@NotNull Servo servo, @NotNull ServoMotionProfile profile) throws InterruptedException {
        double lastAng = servo.getPosition();
        long startTime = System.nanoTime();
        int i = 0;
        for(ServoMotionSegment segment : profile.getSegments()) {
            ServoMotionSegment nextSegment = profile.getSegment(i + 1);
            servo.setPosition(segment.getAngle());
            Thread.sleep((long)nextSegment.getDt() - (long)segment.getDt());
            i++;
        }
    }





}
