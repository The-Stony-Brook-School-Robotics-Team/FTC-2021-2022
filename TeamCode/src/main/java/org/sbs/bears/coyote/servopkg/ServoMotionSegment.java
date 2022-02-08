package org.sbs.bears.coyote.servopkg;

public class ServoMotionSegment {

    private double angle;
    private int dt;

    /**
     * Motion Segment
     * @param angle
     * @param dt
     */
    public ServoMotionSegment(double angle, int dt){
        this.angle = angle;
        this.dt = dt;
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
                this.dt + segment.dt
        );
    }

    public String toString()
    {
        return "ServoMotionSegment with iniAngle " + angle + ", and ∆t " + dt;
    }


}
