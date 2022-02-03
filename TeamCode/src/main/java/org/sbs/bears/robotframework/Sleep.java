package org.sbs.bears.robotframework;

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
