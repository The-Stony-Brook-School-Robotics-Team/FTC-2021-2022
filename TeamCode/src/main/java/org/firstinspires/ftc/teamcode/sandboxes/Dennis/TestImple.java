package org.firstinspires.ftc.teamcode.sandboxes.Dennis;


import org.sbs.bears.coyote.servopkg.ServoMotionProfile;
import org.sbs.bears.coyote.servopkg.ServoMotionSegment;

import java.util.ArrayList;
import java.util.List;

public class TestImple {

    ServoMotionSegment segment1 = new ServoMotionSegment(1, 1, 1, 1);
    ServoMotionSegment segment2 = new ServoMotionSegment(1, 1, 1, 1);
    ServoMotionSegment segment3 = new ServoMotionSegment(1, 1, 1, 1);
    ServoMotionSegment segment4 = new ServoMotionSegment(1, 1, 1, 1);
    ServoMotionSegment segment5 = new ServoMotionSegment(1, 1, 1, 1);

    List<ServoMotionSegment> segments = new ArrayList<>();

    public void staewnlksjnd() {
        ServoMotionSegment segment = segment1.add(segment2);

        ServoMotionProfile prof = new ServoMotionProfile(segments);



    }

}
