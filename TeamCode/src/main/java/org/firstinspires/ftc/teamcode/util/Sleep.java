package org.firstinspires.ftc.teamcode.util;

public class Sleep {
    public static void sleep(long milis)
    {
        try {
            Thread.sleep(milis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
