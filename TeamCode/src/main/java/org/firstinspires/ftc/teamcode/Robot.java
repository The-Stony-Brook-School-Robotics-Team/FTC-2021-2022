package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;

public class Robot {
    // HashMaps
    public static HashMap<String, DcMotor> MotorMap = new HashMap<>();
    public static HashMap<String, DcMotor> EncoderMap = new HashMap<>();
    public static HashMap<String, MotorEx> OdomMap = new HashMap<>();

    // Variables
    private static HardwareMap hardwareMap;
    private static Telemetry telemetry;

    // Motors
    public static DcMotor lf;
    public static DcMotor rf;
    public static DcMotor lb;
    public static DcMotor rb;

    // Odom
    private static HolonomicOdometry holOdom;
    private static MotorEx lOdom;
    private static MotorEx rOdom;
    private static MotorEx bOdom;

    // Encoders
    public static DcMotor lfencoder;
    public static DcMotor rfencoder;
    public static DcMotor lbencoder;
    public static DcMotor rbencoder;

    // Imu
    public static BNO055IMU imu;
    BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
    
    public Robot(@NotNull HardwareMap hardwareMap, @NotNull Telemetry telemetry)
    {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "leftodom");
        lb = hardwareMap.get(DcMotor.class, "backodom");
        rb = hardwareMap.get(DcMotor.class, "rightodom");
        MotorMap.put("lf", lf);
        MotorMap.put("rf", rf);
        MotorMap.put("lb", lb);
        MotorMap.put("rb", rb);

        lOdom = new MotorEx(hardwareMap, "leftodom");
        rOdom = new MotorEx(hardwareMap, "rightodom");
        bOdom = new MotorEx(hardwareMap, "backodom");
        OdomMap.put("lOdom", lOdom);
        OdomMap.put("rOdom", rOdom);
        OdomMap.put("bOdom", bOdom);

        lOdom.setDistancePerPulse(RobotConfig.TICKS_TO_INCHES);
        rOdom.setDistancePerPulse(RobotConfig.TICKS_TO_INCHES);
        bOdom.setDistancePerPulse(RobotConfig.TICKS_TO_INCHES);

        lfencoder = hardwareMap.get(DcMotor.class, "lfencoder");
        rfencoder = hardwareMap.get(DcMotor.class, "rfencoder");
        lbencoder = hardwareMap.get(DcMotor.class, "lbencoder");
        rbencoder = hardwareMap.get(DcMotor.class, "rbencoder");
        EncoderMap.put("lfencoder", lfencoder);
        EncoderMap.put("rfencoder", rfencoder);
        EncoderMap.put("lbencoder", lbencoder);

        holOdom = new HolonomicOdometry(
                lOdom::getDistance,
                rOdom::getDistance,
                bOdom::getDistance,
                RobotConfig.TRACK_WIDTH,
                RobotConfig.CENTER_WHEEL_OFFSET
        );

        lfencoder.setDirection(DcMotorSimple.Direction.REVERSE);
        lbencoder.setDirection(DcMotorSimple.Direction.REVERSE);

        lfencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);
    }

    public HashMap getEncoderMap()
    {
        return EncoderMap;
    }

    public HashMap getMotorMap()
    {
        return MotorMap;
    }

    public HashMap getOdomMap()
    {
        return OdomMap;
    }
}
