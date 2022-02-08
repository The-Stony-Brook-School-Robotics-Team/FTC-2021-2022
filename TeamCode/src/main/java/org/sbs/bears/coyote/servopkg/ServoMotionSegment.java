package org.sbs.bears.coyote.servopkg;

public class ServoMotionSegment {

    public double getIniAng() {
        return iniAng;
    }

    public double getFiniAng() {
        return finiAng;
    }

    public int getDt() {
        return dt;
    }

    public double getV() {
        return v;
    }

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
