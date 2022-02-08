package org.sbs.bears.coyote.servopkg;

import java.util.ArrayList;
import java.util.List;

public class ServoMotionProfile {
    private List<ServoMotionSegment> segments;

    public ServoMotionProfile(List<ServoMotionSegment> segments)
    {
        this.segments = segments;
    }

    public ServoMotionSegment getSegment(int index)
    {
        return segments.get(index);
    }
    public double totalTime()
    {
        double t = 0;
        for (ServoMotionSegment segment : segments) {
                t+= segment.getDt();
        }
        return t;
    }
    public ServoMotionProfile add(ServoMotionProfile otherProf)
    {
        for (ServoMotionSegment segment : otherProf.segments) {
            segments.add(segment);
        }
        return this;
    }
}
