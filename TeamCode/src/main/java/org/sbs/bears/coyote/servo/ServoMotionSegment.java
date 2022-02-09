package org.sbs.bears.coyote.servo;

public class ServoMotionSegment {

    private double angle;
    private double dt;

    /**
     * Motion Segment
     * @param angle angle (clipped range)
     * @param dt delta time in seconds
     */
    public ServoMotionSegment(double angle, double dt){
        this.angle = angle;
        this.dt = dt;
    }

    public double getAngle() {
        return angle;
    }

    public double getDt() {
        return dt;
    }

    public double setAngle(double angle) {
        this.angle = angle;
        return this.angle;
    }

    public double setDt(double dt) {
        this.dt = dt;
        return this.dt;
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
