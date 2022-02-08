package org.sbs.bears.coyote.servopkg;

public class ServoMotionSegment {

    private double angle;
    private int dt;
    private double v;

    /**
     * Motion Segment
     * @param angle
     * @param dt
     * @param v
     */
    public ServoMotionSegment(double angle, int dt, double v){
        this.angle = angle;
        this.dt = dt;
        this.v = v;
    }

    public double getAngle() {
        return angle;
    }

    public int getDt() {
        return dt;
    }

    public double setAngle(double angle) {
        this.angle = angle;
        return angle;
    }

    public int setDt(int add) {
        return dt;
    }


    public ServoMotionSegment add(ServoMotionSegment segment) {
        return new ServoMotionSegment(
                this.angle + segment.angle,
                this.dt + segment.dt,
                this.v + segment.v);
    }

    public String toString()
    {
        return "ServoMotionSegment with iniAngle " + angle + ", finAng " + angle + ", vel " + v + ", and ∆t " + dt;
    }


}
