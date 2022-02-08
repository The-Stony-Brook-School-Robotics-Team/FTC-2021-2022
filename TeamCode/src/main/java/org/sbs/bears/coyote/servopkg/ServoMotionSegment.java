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

    public double setIniAng(double add) {
        return iniAng;
    }

    public double setFiniAng(double add) {
        return finiAng;
    }

    public int setDt(int add) {
        return dt;
    }

    public double setV(double add) {
        return v;
    }

    public ServoMotionSegment add(ServoMotionSegment segment) {
        return new ServoMotionSegment(
                this.iniAng + segment.iniAng,
                this.finiAng + segment.finiAng,
                this.dt + segment.dt,
                this.v + segment.v);
    }
}
