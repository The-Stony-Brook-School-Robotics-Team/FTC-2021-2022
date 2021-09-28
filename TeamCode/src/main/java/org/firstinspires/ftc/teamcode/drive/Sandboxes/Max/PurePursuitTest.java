package org.firstinspires.ftc.teamcode.drive.Sandboxes.Max;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.InterruptWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.function.DoubleSupplier;


public class PurePursuitTest extends LinearOpMode {
    private Motor.Encoder LeftEncoder;
    private Motor.Encoder RightEncoder;
    private Motor.Encoder CentralEncoder;
    private HolonomicOdometry odometry;
    private MotorEx lf = new MotorEx(hardwareMap, "lf"), rf = new MotorEx(hardwareMap, "leftodom"), lb = new MotorEx(hardwareMap, "backodom"), rb = new MotorEx(hardwareMap, "rightodom");

    private final double DistancePerPulse = Math.PI*2.0/8192;
    @Override
    public void runOpMode() throws InterruptedException {
        LeftEncoder = rf.encoder.setDistancePerPulse(DistancePerPulse);
        RightEncoder = rb.encoder.setDistancePerPulse(DistancePerPulse);
        CentralEncoder = lb.encoder.setDistancePerPulse(DistancePerPulse);
        LeftEncoder.reset();
        RightEncoder.reset();
        CentralEncoder.reset();
        DoubleSupplier LP,RP,CP;
        double TicksPerInch = 8192*4*Math.PI;
        LP = () -> LeftEncoder.getPosition()/(TicksPerInch);
        RP = () -> RightEncoder.getPosition()/(TicksPerInch);
        CP = () -> CentralEncoder.getPosition()/(TicksPerInch);

        odometry = new HolonomicOdometry(LP,RP,CP,12.75, -8.7);
        Waypoint startW = new StartWaypoint(LeftEncoder.getPosition(),RightEncoder.getPosition());
        //Waypoint premW = new InterruptWaypoint(8192*2, 8192*2, odometry.updatePose()); //Learning "Position Buffer"
        Waypoint intermediateW= new GeneralWaypoint(8192*10, 8192*10, Math.PI*2, 1,1, 2*Math.PI);
        Waypoint postW = new InterruptWaypoint();
        Waypoint endW = new EndWaypoint();
        Path testP = new Path(startW,intermediateW);
        testP.init();
    }
}
