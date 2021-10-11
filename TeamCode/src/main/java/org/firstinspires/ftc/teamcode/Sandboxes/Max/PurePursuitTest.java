package org.firstinspires.ftc.teamcode.Sandboxes.Max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sandboxes.Dennis.Odometry.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.function.DoubleSupplier;

@TeleOp
public class PurePursuitTest extends LinearOpMode {
//    private Motor.Encoder LeftEncoder;
//    private Motor.Encoder RightEncoder;
//    private Motor.Encoder CentralEncoder;
    private HolonomicOdometry HolonomicOdometry;
    private OdometrySubsystem OdometrySubSystem;

    private MotorEx LeftEncoder;
    private MotorEx RightEncoder;
    private MotorEx CentralEncoder;

    private MotorEx lf;
    private MotorEx rf;
    private MotorEx lb;
    private MotorEx rb;
    private MecanumDrive Drivers;
    FtcDashboard Graph;

    private final double DistancePerPulse = Math.PI*2.0/8192;
    @Override
    public void runOpMode() throws InterruptedException {
         lf = new MotorEx(hardwareMap, "lf");
         rf = new MotorEx(hardwareMap, "leftodom");
         lb = new MotorEx(hardwareMap, "backodom");
         rb = new MotorEx(hardwareMap, "rightodom");

        LeftEncoder = new MotorEx(hardwareMap, "leftodom");
        CentralEncoder = new MotorEx(hardwareMap, "backodom");
        RightEncoder = new MotorEx(hardwareMap, "rightodom");

        Drivers = new MecanumDrive(lf, rf, lb, rb);
        //Drivers;
        LeftEncoder.setDistancePerPulse(DistancePerPulse);
        RightEncoder.setDistancePerPulse(DistancePerPulse);
        CentralEncoder.setDistancePerPulse(DistancePerPulse);
        LeftEncoder.set(0);
        RightEncoder.set(0);
        CentralEncoder.set(0);



        DoubleSupplier LP,RP,CP;
        double TicksPerInch = 8192*4*Math.PI;
        LP = () -> LeftEncoder.getCurrentPosition()/(TicksPerInch);
        RP = () -> -(RightEncoder.getCurrentPosition()/(TicksPerInch));
        CP = () -> CentralEncoder.getCurrentPosition()/(TicksPerInch);


        HolonomicOdometry = new HolonomicOdometry(LP,RP,CP,12.75, -8.7);
        OdometrySubSystem = new OdometrySubsystem(this.HolonomicOdometry);


        FtcDashboard Dashboard = FtcDashboard.getInstance();
        TelemetryPacket TelemetryPacket = new TelemetryPacket();
        Canvas Field = new TelemetryPacket().fieldOverlay();
        com.acmerobotics.roadrunner.geometry.Pose2d Pose2dField= new com.acmerobotics.roadrunner.geometry.Pose2d(OdometrySubSystem.getPose().getX(), OdometrySubSystem.getPose().getY(),OdometrySubSystem.getPose().getHeading());
        waitForStart();

        Waypoint startW = new StartWaypoint(LeftEncoder.getCurrentPosition(), RightEncoder.getCurrentPosition());
        //Waypoint premW = new InterruptWaypoint(8192*2, 8192*2, odometry.updatePose()); //Learning "Position Buffer"
        Waypoint intermediateW = new GeneralWaypoint(LeftEncoder.getCurrentPosition()+8192 * 5, RightEncoder.getCurrentPosition()+ 8192* 5, Math.PI * 2, 1, 50, 2 * Math.PI);
        Waypoint postW = new InterruptWaypoint();
        Waypoint endW = new EndWaypoint(LeftEncoder.getCurrentPosition()+8192 * 11, RightEncoder.getCurrentPosition()+ 8192* 11, Math.PI * 2, 1, 50, 2 * Math.PI * 2, 2 * Math.PI * 2, 2 * Math.PI * 2);
        Path testP = new Path(startW, intermediateW, endW);
        testP.setWaypointTimeouts(100);
        testP.init();

        while(opModeIsActive() && !isStopRequested()) {

            OdometrySubSystem.update();
            TelemetryPacket.put("Pure Pursuit Position Indicator", 0);
            TelemetryPacket.put("X: ", OdometrySubSystem.getPose().getX());
            TelemetryPacket.put("Y: ", OdometrySubSystem.getPose().getY());
            TelemetryPacket.put("H: ", OdometrySubSystem.getPose().getHeading());
            Dashboard.sendTelemetryPacket(TelemetryPacket);
            telemetry.addData("X: ", LeftEncoder.getCurrentPosition());
            telemetry.addData("Y: ", RightEncoder.getCurrentPosition());
            telemetry.addData("H: ", CentralEncoder.getCurrentPosition());
            telemetry.update();

            if (gamepad1.a) {
                LeftEncoder.set(0);
                RightEncoder.set(0);
                CentralEncoder.set(0);
            }

            if (gamepad1.y) {
                wait(100);
                /*The Code Below Is Related to The Pure Pursuit Road Tracking Codes Above*/
                testP.followPath(Drivers, HolonomicOdometry);
            }

            lf.set((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            rf.set((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            lb.set((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            rb.set((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));

            DashboardUtil.drawRobot(Field, Pose2dField);
        }


    }

}