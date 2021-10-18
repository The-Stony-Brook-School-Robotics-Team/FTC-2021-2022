package org.firstinspires.ftc.teamcode.Sandboxes.Max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private SampleMecanumDrive ControlledDrives;
    private int ButtonY = 0;
    FtcDashboard Graph;

    private final double DistancePerPulse = Math.PI*2.0/8192;
    @Override
    public void runOpMode() throws InterruptedException {
        ControlledDrives = new SampleMecanumDrive(hardwareMap);
        ControlledDrives.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         lf = new MotorEx(hardwareMap, "lf");
         rf = new MotorEx(hardwareMap, "leftodom");
         lb = new MotorEx(hardwareMap, "backodom");
         rb = new MotorEx(hardwareMap, "rightodom");

        LeftEncoder = new MotorEx(hardwareMap, "leftodom");
        CentralEncoder = new MotorEx(hardwareMap, "backodom");
        RightEncoder = new MotorEx(hardwareMap, "rightodom");

        Drivers = new MecanumDrive(lf, rf, lb, rb);
        ControlledDrives = new SampleMecanumDrive(hardwareMap);
        //Drivers;
        LeftEncoder.setDistancePerPulse(DistancePerPulse);
        RightEncoder.setDistancePerPulse(DistancePerPulse);
        CentralEncoder.setDistancePerPulse(DistancePerPulse);
        LeftEncoder.set(0);
        RightEncoder.set(0);
        CentralEncoder.set(0);





        while(true) {

            DoubleSupplier LP,RP,CP;
            double TicksPerInch = 8192*4*Math.PI;
            LP = () -> LeftEncoder.getCurrentPosition()/(TicksPerInch);
            RP = () -> (RightEncoder.getCurrentPosition()/(TicksPerInch));
            CP = () -> CentralEncoder.getCurrentPosition()/(TicksPerInch);


            HolonomicOdometry = new HolonomicOdometry(LP,RP,CP,12.75, -8.7);
            OdometrySubSystem = new OdometrySubsystem(this.HolonomicOdometry);


            waitForStart();

            Waypoint startW = new StartWaypoint(OdometrySubSystem.getPose().getX(), OdometrySubSystem.getPose().getY());
            //Waypoint premW = new InterruptWaypoint(8192*2, 8192*2, odometry.updatePose()); //Learning "Position Buffer"
            Waypoint intermediateW = new GeneralWaypoint(OdometrySubSystem.getPose().getX()+8192 * 5, 0);
            Waypoint postW = new InterruptWaypoint();
            //Waypoint endW = new EndWaypoint(LeftEncoder.getCurrentPosition()+8192 * 11, 0, Math.PI/4, 0.6, 0.2, Math.PI, Math.PI, Math.PI );
            Waypoint endW = new EndWaypoint(OdometrySubSystem.getPose().getX(), 0, 0, 0.3, 0.3, Math.PI, Math.PI*10, Math.PI*10);
            Path testP = new Path(startW, endW);
            //testP.setWaypointTimeouts(100);


            Graph = FtcDashboard.getInstance();
            TelemetryPacket TelemetryPacket = new TelemetryPacket();
            Canvas Field = new TelemetryPacket().fieldOverlay();
            Pose2d Pose2dField= new Pose2d(OdometrySubSystem.getPose().getX(), OdometrySubSystem.getPose().getY(),OdometrySubSystem.getPose().getHeading());
            Graph.updateConfig();
            OdometrySubSystem.update();

            TelemetryPacket.put("Pure Pursuit Position Indicator", 1);
            TelemetryPacket.put("X: ", OdometrySubSystem.getPose().getX());
            TelemetryPacket.put("Y: ", OdometrySubSystem.getPose().getY());
            TelemetryPacket.put("H: ", OdometrySubSystem.getPose().getHeading());
            Graph.sendTelemetryPacket(TelemetryPacket);
            telemetry.addData("X: ", LeftEncoder.getCurrentPosition());
            telemetry.addData("Y: ", RightEncoder.getCurrentPosition());
            telemetry.addData("H: ", Math.toDegrees(CentralEncoder.getCurrentPosition()));
            telemetry.update();
            DashboardUtil.drawRobot(Field, Pose2dField);

            if (gamepad1.a) {
                LeftEncoder.resetEncoder();
                RightEncoder.resetEncoder();
                CentralEncoder.resetEncoder();
            }


            if (gamepad1.y && ButtonY!=1) {
                /*The Code Below Is Related to The Pure Pursuit Road Tracking Codes Above*/
                ButtonY = 1;
            }
            else if(!gamepad1.y && ButtonY == 1){
                testP.init();
                testP.followPath(Drivers, HolonomicOdometry);
                ButtonY = 0;
            }

            ControlledDrives.setWeightedDrivePower(
                    new Pose2d(
                            -0.3*gamepad1.left_stick_x,
                            -0.3*gamepad1.left_stick_y,
                            -0.3*gamepad1.right_stick_x
                    )
            );

        }


    }

}