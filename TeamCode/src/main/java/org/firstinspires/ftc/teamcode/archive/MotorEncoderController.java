package org.firstinspires.ftc.teamcode.archive;

import android.opengl.Matrix;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.NoDataException;
import org.apache.commons.math3.exception.NotPositiveException;
import org.apache.commons.math3.exception.NotStrictlyPositiveException;
import org.apache.commons.math3.exception.NullArgumentException;
import org.apache.commons.math3.exception.NumberIsTooSmallException;
import org.apache.commons.math3.exception.OutOfRangeException;
import org.apache.commons.math3.linear.MatrixDimensionMismatchException;
import org.apache.commons.math3.linear.NonSquareMatrixException;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealMatrixChangingVisitor;
import org.apache.commons.math3.linear.RealMatrixPreservingVisitor;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class controls the motors and encoders, initializing their directions and providing
 * common functions such as simple movements and actions.
 */

public class MotorEncoderController {
    public static String[] motorNames = new String[]{"rightodom","backodom","leftodom","lf"}; // rb lb rf lf
    public static String[] odomNames = new String[]{"rightodom","backodom","leftodom"};
    public static String[] motorEncoderNames = new String[]{"rbencoder","lbencoder","rfencoder","lfencoder"};
    DcMotor[] motors = new DcMotor[4];
    DcMotor[] odoms = new DcMotor[3];
    DcMotor[] motorEncoders = new DcMotor[4];

    Pose2d robotPos;

    public static  double LATERAL_DISTANCE = 8.45;
    public static  double FORWARD_OFFSET = -9.75;
    public static  double CONSTANT = 1;

    double lastRodom, lastLodom, lastBodom;



    double[] odomValsSoft = new double[3];
    double[] odomDiffs = new double[3];

    double[] encoderValsSoft = new double[4];
    double[] encoderDiffs = new double[4];

    static final double odomRadius = 1;

    public static double kPmotorEncoderPID = 1;
    public static double kImotorEncoderPID = 0;
    public static double kDmotorEncoderPID = 0;

    static final double TICKS_TO_INCHES = 8192*odomRadius*2*Math.PI;

    private Telemetry telemetry;
    public static final double TRACKWIDTH = 12.75;
    static final double CENTER_WHEEL_OFFSET = 2.4;

    public MotorEncoderController(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
       robotPos = new Pose2d();
        for (int i = 0; i < 4; i++) {
            motors[i] = (hwMap.get(DcMotor.class, motorNames[i]));
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motorEncoders[i] = (hwMap.get(DcMotor.class, motorEncoderNames[i]));

            /*if(i==0 || i== 3) {
                motorEncoders[i].setDirection(DcMotorSimple.Direction.REVERSE);
            }*/



        }
        // rb lb rf lf
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        motorEncoders[3].setDirection(DcMotorSimple.Direction.REVERSE);
        motorEncoders[1].setDirection(DcMotorSimple.Direction.REVERSE);



        for (int i = 0; i < 3; i++) {
            odoms[i] = hwMap.get(DcMotor.class, odomNames[i]);
            odomValsSoft[i] = 0;
            odomDiffs[i] = odoms[i].getCurrentPosition() - odomValsSoft[i];
            encoderValsSoft[i] = 0;
            encoderDiffs[i] = motorEncoders[i].getCurrentPosition() - encoderValsSoft[i];
        }
        encoderValsSoft[3] = 0;
        encoderDiffs[3] = motorEncoders[3].getCurrentPosition() - encoderValsSoft[3];
        lastBodom = Bodom().getCurrentPosition()/4;
        lastRodom = Rodom().getCurrentPosition()/4;
        lastLodom = Lodom().getCurrentPosition()/4;
    }
    public DcMotor[] getMotors() {
        return motors;
    }

    public DcMotor LF() {
        return motors[2];
    }
    public DcMotor RF() {
        return motors[3];
    }
    public DcMotor LB() {
        return motors[1];
    }
    public DcMotor RB() { return motors[0]; }

    public DcMotor[] getMotorEncoders() {
        return motorEncoders;
    }

    public DcMotor LFencoder() { return motorEncoders[2]; }
    public DcMotor RFencoder() {
        return motorEncoders[3];
    }
    public DcMotor LBencoder() {
        return motorEncoders[1];
    }
    public DcMotor RBencoder() { return motorEncoders[0]; }

    public DcMotor[] getOdoms() {
        return odoms;
    }


    public DcMotor Lodom() {
        return odoms[2];
    }
    public DcMotor Rodom() {
        return odoms[0];
    }
    public DcMotor Bodom() {
        return odoms[1];
    }

    public double[] getOdomValsSoft() {
        for (int i = 0; i < 3; i++) {
            odomValsSoft[i] = odoms[i].getCurrentPosition() - odomDiffs[i];
        }
        return odomValsSoft;
    }

    public double getLOdomValSoft() {
        return getOdomValsSoft()[2];
    }
    public double getROdomValSoft() {
        return getOdomValsSoft()[0];
    }
    public double getBOdomValSoft() {
        return getOdomValsSoft()[1];
    }

    public void resetSoftOdom() {
        for(int i = 0; i < 3; i++) {
            odomDiffs[i] = odoms[i].getCurrentPosition();
            odomValsSoft[i] = 0;
        }
    }

    // recall order
    // rb lb rf lf
    public double[] getEncoderValsSoft() {
        for (int i = 0; i < 4; i++) {
            //System.out.println("GettingSoftVal " + i);
            encoderValsSoft[i] = motorEncoders[i].getCurrentPosition() - encoderDiffs[i];
        }
        return encoderValsSoft;
    }

    public double getLFEncoderValSoft() {
        ///.out.println("GettingSoftVal LF");
        return getEncoderValsSoft()[3];
    }
    public double getRFEncoderValSoft() {
        //System.out.println("GettingSoftVal RF");
        return getEncoderValsSoft()[2];
    }
    public double getLBEncoderValSoft() {
        //System.out.println("GettingSoftVal LB");
        return getEncoderValsSoft()[1];
    }
    public double getRBEncoderValSoft() {
        //System.out.println("GettingSoftVal RB");
        return getEncoderValsSoft()[0]; }

    public void resetSoftMotorEncoders() {
        for(int i = 0; i < 4; i++) {
            encoderDiffs[i] = motorEncoders[i].getCurrentPosition();
            encoderValsSoft[i] = 0;
        }
    }




    public static double convertOdomTickToRobotInches(double ticks) {
        return ticks/(8192)*(odomRadius*2*Math.PI) ;
    }
    public static double convertRobotInchesToTicks(double inches) {
        return inches*(8192)/(odomRadius*2*Math.PI) ;
    }



    public void goForwardPower(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public void goBackwardPower(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(-power);
        }
    }

    public void turnRightPower(double power) {
        LF().setPower(power);
        RF().setPower(-power);
        LB().setPower(-power);
        RB().setPower(power);

    }
    public void turnLeftPower(double power) {
        power = -power;
        LF().setPower(power);
        RF().setPower(-power);
        LB().setPower(-power);
        RB().setPower(power);
    }

    public void strafeLeftPower(double power) {
        LF().setPower(-power);
        RF().setPower(power);
        LB().setPower(-power);
        RB().setPower(power);
    }
    public void strafeRightPower(double power) {
        LF().setPower(power);
        RF().setPower(-power);
        LB().setPower(power);
        RB().setPower(-power);
    }
    public void diffPower(double powerL, double powerR) {
        LF().setPower(powerL);
        RF().setPower(powerR);
        LB().setPower(powerR);
        RB().setPower(powerL);
    }


    public void stopRobot() {
        goForwardPower(0);
    }


    public void goForwardDistPID(double dist) {
        double power = 0.4;
        resetSoftOdom();
        dist = convertRobotInchesToTicks(dist);
        telemetry.addData("L ODOM",getLOdomValSoft());
        telemetry.addData("R ODOM",getROdomValSoft());
        telemetry.addData("B ODOM",getBOdomValSoft());
        telemetry.addData("Lpow",LF().getPower());
        telemetry.addData("Rpow",RF().getPower());
        telemetry.addData("powerRatio",LF().getPower()/RF().getPower());
        telemetry.addData("odomDiff",getROdomValSoft() - getLOdomValSoft());
        telemetry.addData("TO Travel",dist);
        double k = 0.001;
        while(getLOdomValSoft() <= dist) {
            double odomDiff = getBOdomValSoft();
            System.out.println("odomDiff: " + odomDiff);
            // we want Lpow/Rpow = 1 + k * odomDiff
            double rPow = power;
            double lPow = power*(1+k*odomDiff);



            System.out.println("lPow: " + lPow);
            diffPower(lPow,rPow);
            telemetry.addData("L ODOM",getLOdomValSoft());
            telemetry.addData("R ODOM",getROdomValSoft());
            telemetry.addData("B ODOM",getBOdomValSoft());
            telemetry.addData("Lpow",LF().getPower());
            telemetry.addData("Rpow",RF().getPower());
            telemetry.addData("powerRatio",LF().getPower()/RF().getPower());
            telemetry.addData("odomDiff",getROdomValSoft() - getLOdomValSoft());
            telemetry.addData("TO Travel",dist);

            telemetry.update();
        }
        stopRobot();
    }


    public void goForwardDist(double dist) {
        if (dist < 0.1) {return;}
        double power = 0.4;
        resetSoftOdom();
        telemetry.addData("L ODOM",getLOdomValSoft());
        telemetry.addData("R ODOM",getROdomValSoft());
        telemetry.addData("B ODOM",getBOdomValSoft());
        telemetry.addData("Lpow",LF().getPower());
        telemetry.addData("Rpow",RF().getPower());
        telemetry.addData("powerRatio",LF().getPower()/RF().getPower());
        telemetry.addData("odomDiff",getROdomValSoft() - getLOdomValSoft());
        telemetry.addData("TO Travel",1000);

        telemetry.update();
        while(getLOdomValSoft() <= dist) {
            goForwardPower(power);
            telemetry.addData("L ODOM",getLOdomValSoft());
            telemetry.addData("R ODOM",getROdomValSoft());
            telemetry.addData("B ODOM",getBOdomValSoft());
            telemetry.addData("Lpow",LF().getPower());
            telemetry.addData("Rpow",RF().getPower());
            telemetry.addData("powerRatio",LF().getPower()/RF().getPower());
            telemetry.addData("odomDiff",getROdomValSoft() - getLOdomValSoft());
            telemetry.addData("TO Travel",1000);

            telemetry.update();
        }
        stopRobot();
    }

    public void goBackwardDist(double dist) {
        if (dist < 0.1) {return;}
        double power = 0.4;
        resetSoftOdom();
        telemetry.addData("L ODOM",getLOdomValSoft());
        telemetry.addData("R ODOM",getROdomValSoft());
        telemetry.addData("B ODOM",getBOdomValSoft());
        telemetry.addData("Lpow",LF().getPower());
        telemetry.addData("Rpow",RF().getPower());
        telemetry.addData("powerRatio",LF().getPower()/RF().getPower());
        telemetry.addData("odomDiff",getROdomValSoft() - getLOdomValSoft());
        telemetry.addData("TO Travel",1000);

        telemetry.update();
        while(getLOdomValSoft() >= dist) {
            goForwardPower(-power);
            telemetry.addData("L ODOM",getLOdomValSoft());
            telemetry.addData("R ODOM",getROdomValSoft());
            telemetry.addData("B ODOM",getBOdomValSoft());
            telemetry.addData("Lpow",LF().getPower());
            telemetry.addData("Rpow",RF().getPower());
            telemetry.addData("powerRatio",LF().getPower()/RF().getPower());
            telemetry.addData("odomDiff",getROdomValSoft() - getLOdomValSoft());
            telemetry.addData("TO Travel",1000);

            telemetry.update();
        }
        stopRobot();
    }

    public void turnRightDeg(double deg) {
        double power = 0.4;
        LF().setPower(power);
        RF().setPower(-power);
        LB().setPower(-power);
        RB().setPower(power);

    }
    public void turnLeftDeg(double deg) {
        double power = 0.4;
        power = -power;
        LF().setPower(power);
        RF().setPower(-power);
        LB().setPower(-power);
        RB().setPower(power);
    }

    public void strafeLeftDist(double dist) {
        if (dist < 0.1) {return;}
        double power = 0.45;
        resetSoftOdom();
        while(getBOdomValSoft() >= (dist)) {
            strafeLeftPower(power);
        }
        stopRobot();
    }
    public void strafeRightDist(double dist) {
        if (dist < 0.1) {return;}
        double power = 0.45;
        resetSoftOdom();
        while(getBOdomValSoft() <= (dist)) {
            strafeRightPower(power);
        }
        stopRobot();
    }



    public Pose2d getPosition() {
        double deltaL = Lodom().getCurrentPosition() - lastLodom;
        double deltaR = Rodom().getCurrentPosition() - lastRodom;
        double deltaB = Bodom().getCurrentPosition() - lastBodom;

        deltaL = convertOdomTickToRobotInches(deltaL);
        deltaR = convertOdomTickToRobotInches(deltaR);
        deltaB = convertOdomTickToRobotInches(deltaB);

        double deltaAngle = (deltaL - deltaR)/LATERAL_DISTANCE;
        //System.out.println("\n\n");
        //System.out.println(deltaAngle*LATERAL_DISTANCE);
        //System.out.println(deltaAngle);
        double forwardDisp = (deltaL + deltaR)/2;

        double sideDisp = deltaB - (FORWARD_OFFSET*deltaAngle);

        double heading0 = robotPos.getHeading();
        double dH = -deltaAngle;
        //double newY = -(forwardDisp*Math.sin(heading0) + sideDisp*Math.cos(heading0))*1.2517 + robotPos.getY();
        //double newX = (forwardDisp*Math.cos(heading0) - sideDisp*Math.sin(heading0))*1.2517 + robotPos.getX()
        double dX = (forwardDisp*Math.cos(heading0) - sideDisp*Math.sin(heading0));
        double dY = (forwardDisp*Math.sin(heading0) + sideDisp*Math.cos(heading0));

        print("forward Disp: " + forwardDisp);
        print("sideDisp: " + sideDisp);
        print("dH: " + dH);
        print("dX: " + dX);
        print("dY: " + dY);

        double adjX = dH == 0 ? dX : dX*Math.sin(dH)/dH+(Math.cos(dH)-1)/dH*dY;
        double adjY = dH == 0 ? dY : dX * (1 - Math.cos(dH))/dH  + dY * Math.sin(dH)/dH;
        double adjH = dH;
        print("adjX: " + adjX);
        print("adjY: " + adjY);
        print("adjH: " + adjH);
        robotPos = new Pose2d(adjX + robotPos.getX(),adjY + robotPos.getY(),new Rotation2d(adjH));

        lastBodom = Bodom().getCurrentPosition();
        lastRodom = Rodom().getCurrentPosition();
        lastLodom = Lodom().getCurrentPosition();
        return robotPos;
    }
    public Pose2d resetPosition() {
        return this.resetPosition(new Pose2d());
    }
    public Pose2d resetPosition(Pose2d newPos) {
        robotPos = newPos;
        lastBodom = Bodom().getCurrentPosition();
        lastRodom = Rodom().getCurrentPosition();
        lastLodom = Lodom().getCurrentPosition();
        return robotPos;
    }

    public void print(String s) {
        System.out.println(s);
    }

}
