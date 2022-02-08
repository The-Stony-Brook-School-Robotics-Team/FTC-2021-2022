package org.sbs.bears.coyote.servopkg;

public class ServoMotionSegment {

    private double iniAng;
    private double finiAng;
    private int dt;
    private double v;

    /**
     * Motion Segment
     * @param iniAng
     * @param finalAng
     * @param dt
     * @param v
     */
    public ServoMotionSegment(double iniAng, double finalAng, int dt, double v){
        this.iniAng = iniAng;
        this.finiAng = finalAng;
        this.dt = dt;
        this.v = v;
    }

}
