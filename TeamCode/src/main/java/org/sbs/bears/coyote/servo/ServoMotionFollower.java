package org.sbs.bears.coyote.servo;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.teleop.misc.Beta;
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
     */
    @Beta
    @DoNotUse
    public ServoMotionProfile genTest(double step, double max) {
        ArrayList<ServoMotionSegment> segmentList = new ArrayList<>();
        for(double x = 0; x <= max; x += step) {
            double base = 1 + Math.pow(Math.E, -10 * (x - 0.6));
            double y = 1 / base;
            ServoMotionSegment segment = new ServoMotionSegment(y, x);
            segmentList.add(segment);
        }
        return new ServoMotionProfile(segmentList);
    }

    /**
     * Get the y pos based off of time
     * @param x time
     * @return position
     * @version 1
     */
    public double returnY(double x) {
        double base = 1 + Math.pow(Math.E, -10 * (x - 0.6));
        return 1 / base;
    }


}

