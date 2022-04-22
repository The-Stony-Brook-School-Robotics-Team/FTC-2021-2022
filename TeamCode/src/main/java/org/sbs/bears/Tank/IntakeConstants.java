package org.sbs.bears.Tank;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConstants {
    public static double redScooperPosition_DUMP = 0.89;
    public static double redScooperPosition_BASE = 0.3;

    public static double blueScooperPosition_BASE = 0.92;
    public static double blueScooperPosition_DUMP = 0.34;

    public static double clawPosition_BASE = 0.2;
    public static double clawPosition_DUMP = 0.6;

    public static double intakePower_BASE = 1;
    public static double intakePower_DUMP = -1;
    public static double intakePower_PARK = 0;

    public static double freightDetectionThreshold = 30;
    public static double clawFreightDetectionThreshold = 70; //80;
    public static double waitForScooper = 400; //ms, was 200.


}
