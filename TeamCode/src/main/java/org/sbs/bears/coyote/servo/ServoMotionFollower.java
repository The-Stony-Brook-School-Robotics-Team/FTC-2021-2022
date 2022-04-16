package org.sbs.bears.coyote.servo;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.teleop.v1.misc.Beta;
import org.jetbrains.annotations.NotNull;
import org.sbs.bears.coyote.enums.DoNotUse;

import java.util.ArrayList;

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
        long startTime = System.nanoTime();
        int i = 0;
        for(ServoMotionSegment segment : profile.getSegments()) {
            ServoMotionSegment nextSegment = profile.getSegment(i + 1);
            servo.setPosition(segment.getAngle());
            Thread.sleep((long)nextSegment.getDt() - (long)segment.getDt());
            i++;
        }
    }

    /**
     * Simple motion generation for a servo
     * @version 1
     * @param pos the position you want to reach
     * @param step the increment value you want
     * @param time the time you want it to take to reach the position
     * @example pos: 1 step: 0.01 time: 1 second
     */
    @Beta
    @DoNotUse
    public ServoMotionProfile genTest(double pos, double step, double time) {
        ArrayList<ServoMotionSegment> segmentList = new ArrayList<>();
        for(double x = 0; x <= time; x += step) {
            double base = 1 + Math.pow(Math.E, -7 * (x - (time / 2)));
            double y = 1 / base;
            ServoMotionSegment segment = new ServoMotionSegment(y, x);
            segmentList.add(segment);
        }
        return new ServoMotionProfile(segmentList);
    }


}

