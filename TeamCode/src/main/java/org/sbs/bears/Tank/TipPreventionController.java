package org.sbs.bears.Tank;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class TipPreventionController {
    public Servo redPreventer;
    public Servo bluePreventer;
    public static double redPreventerOutOfWay = 0.38;
    public static double bluePreventerOutOfWay = 0.62;
    public static double redPreventerPrevent = 0;
    public static double bluePreventerPrevent = 1;

    public TipPreventionController(HardwareMap hardwareMap)
    {
        redPreventer = hardwareMap.get(Servo.class, "rtr");
        bluePreventer = hardwareMap.get(Servo.class, "btr");

    }
    public void outOfWay()
    {
        redPreventer.setPosition(redPreventerOutOfWay);
        bluePreventer.setPosition(bluePreventerOutOfWay);
    }
    public void prevent()
    {
        redPreventer.setPosition(redPreventerPrevent);
        bluePreventer.setPosition(bluePreventerPrevent);
    }

}
