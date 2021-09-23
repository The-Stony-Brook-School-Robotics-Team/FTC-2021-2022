package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Marc TurnTeleOp", group="drive")
public class TurnTeleOp extends OpMode {

    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private BNO055IMU imu = null;
    boolean pressingX = false;
    private boolean qA = false;
    private boolean qB = false;
    private boolean qX = false;
    private boolean qY = false;
    private Double iniAngle = 0.0;

    private double kP, kI, kD;

    private boolean qPIDon = false;
    private boolean qTurnL = true;

    double currentError = 0;
    double currentTarget = 45;
    double deltaTarget = 5;

    double maxPower = 0.4;
    double minPower = 0.25;

    private final double MAX_ERR_THRESH = 0.5;


    @Override
    public void init() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //lf.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters imu_params = new BNO055IMU.Parameters();

        imu.initialize(imu_params);
        kP = 0.005;
        kI = 0;
        kD = 0;
    }

    public double getHeading(AngleUnit angleUnit) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                angleUnit);
        return angles.firstAngle;
    }
    public double getHeadingVel(AngleUnit angleUnit) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                angleUnit);
        return imu.getAngularVelocity().zRotationRate;
    }

    @Override
    public void loop() {
        double currentHeading = getHeading(AngleUnit.DEGREES);

        if(qPIDon) {
            currentError = Math.abs(iniAngle + currentTarget - currentHeading);
            if(currentError <= MAX_ERR_THRESH) {
                stopMotors();
                qPIDon = false;
            }
            else {
                double percError = 100 * currentError / currentTarget;
                double output    = Math.max(minPower,genSpeedProfile(percError)/100*maxPower);
    //                double output = Math.abs(kP * currentError / currentTarget);

                if(currentTarget > 0) {
                    turnL(output);
                }
                else {
                    turnR(output);
                }
            }
        }
        else {
            iniAngle = currentHeading;
            if(gamepad1.x && !qX && iniAngle + currentTarget < 180 - deltaTarget) {
                qX = true;
                currentTarget += deltaTarget;
            }
            else {
                qX = false;
            }
            if(gamepad1.b && !qB && iniAngle + currentTarget > -180 + deltaTarget) {
                qB = true;
                currentTarget -= deltaTarget;
            }
            else {
                qB = false;
            }
            if(gamepad1.y && !qY && currentTarget != 0 && iniAngle + currentTarget < 180 && iniAngle + currentTarget > -180) {
                qPIDon = true;
            }
            else {
                qY = false;
            }
            if(gamepad1.a && !qA) {
                currentTarget *= -1;
            }
            else {
                qA = false;
            }
        }

        telemetry.addData("Current Heading, Vel",currentHeading + " " + getHeadingVel(AngleUnit.DEGREES));
        telemetry.addData("Ini Heading, Status",iniAngle + " " + qPIDon + " " + currentError + " " + currentTarget);
        //telemetry.addData("Is PIDing",qDidTryPID);
        telemetry.update();
/*
        if(gamepad1.a && !qA) {
            lf.setPower(1); // wrong way
            qA = true;
        }
        if(!gamepad1.a && qA) {
            lf.setPower(0);
            qA = false;
        }
        if(gamepad1.b && !qB) {
            rf.setPower(1);
            qB = true;
        }
        if(!gamepad1.b && qB) {
            rf.setPower(0);
            qB = false;
        }
        if(gamepad1.x && !qX) {
            lb.setPower(1); // wrong way
            qX = true;
        }
        if(!gamepad1.x && qX) {
            lb.setPower(0);
            qX = false;
        }
        if(gamepad1.y && !qY) {
            rb.setPower(1);
            qY = true;
        }
        if(!gamepad1.y && qY) {
            rb.setPower(0);
            qY = false;
        }

*/


/*
        lf.setPower(0.3*(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
        rf.setPower(0.3*(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
        lb.setPower(0.3*(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
        rb.setPower(0.3*(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
*/
    }

    private void doTurnLStopCheck() {
        if(currentError <= 0) {
            stopMotors();
            qPIDon = false;
            iniAngle = 0.0;
        }
    }


    public void turnR(double speed) {
        lf.setPower(speed);
        lb.setPower(speed);
        rf.setPower(-speed);
        rb.setPower(-speed);
    }
    public void turnL(double speed) {
        lf.setPower(-speed);
        lb.setPower(-speed);
        rf.setPower(speed);
        rb.setPower(speed);
    }
    public void stopMotors() {
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
    }

    public double genSpeedProfile(double x) {
       // \frac{d}{2}-\left|\frac{-d}{1+e^{-\left(0.14x-8\right)}}+\frac{d}{2}\right|
        //final double d = 100;
        // -0.0000000064(x*x*x)*(x-100)*(x-100)*(x-100)
        return -0.0000000064*(x*x*x)*(x-100)*(x-100)*(x-100);
        //d/2-Math.abs(-d/(1+Math.exp(-(0.14*x-8)))+d/2);


    }


}
