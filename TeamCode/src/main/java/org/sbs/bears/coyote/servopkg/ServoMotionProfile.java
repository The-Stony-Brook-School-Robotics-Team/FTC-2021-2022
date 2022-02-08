package org.sbs.bears.coyote.servopkg;


import org.sbs.bears.coyote.servopkg.math.Polynomial;

import java.util.ArrayList;
import java.util.List;

public class ServoMotionProfile {
    private List<ServoMotionSegment> segments;

    public ServoMotionProfile(List<ServoMotionSegment> segments)
    {
        this.segments = segments;
    }

    public ServoMotionProfile()
    {
        this.segments = new ArrayList<>();

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

    public ArrayList<ServoMotionSegment> getSegments()
    {
        return new ArrayList<>(segments);
    }
    public ServoMotionProfile add(ServoMotionProfile otherProf)
    {
        for (ServoMotionSegment segment : otherProf.segments) {
            segments.add(segment);
        }
        return this;
    }
    public double getStartAng()
    {
        return segments.get(0).getIniAng();
    }
    public double getEndAng()
    {
        return segments.get(segments.size()-1).getFiniAng();
    }
    public void generateProfile(Polynomial poly, double dt)
    {
        // TODO
    }
    public String toString()
    {
        return "ServoMotionProfile with " + segments.size() + " segments; total time is " + totalTime() + " seconds.";
    }
}
