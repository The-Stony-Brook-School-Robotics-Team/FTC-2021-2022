package org.sbs.bears.robotframework.controllers;

import static org.sbs.bears.robotframework.enums.MotorName.LB;
import static org.sbs.bears.robotframework.enums.MotorName.LF;
import static org.sbs.bears.robotframework.enums.MotorName.MASTER;
import static org.sbs.bears.robotframework.enums.MotorName.MASTER_LOOP;
import static org.sbs.bears.robotframework.enums.MotorName.RB;
import static org.sbs.bears.robotframework.enums.MotorName.RF;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.drive.DriveConstantsMain;
import org.firstinspires.ftc.teamcode.drive.DriveConstantsTank;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.enums.DrivingMode;
import org.sbs.bears.robotframework.enums.MotorName;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

public class DrivingControllerTank {
    private static final int EPSILON = 10;
    public SampleMecanumDrive RR;
    DrivingMode mode;
    AtomicReference<Boolean> isFollowingTraj = new AtomicReference<>();
    Map<MotorName, DcMotorEx> motorMap = new HashMap<>();
    AtomicReference<Map<MotorName, Boolean>> PIDreadySignal = new AtomicReference<>();
    AtomicReference<Map<MotorName, Boolean>> PIDdoneSignal = new AtomicReference<>();
    Map<MotorName, Integer> encoderMap = new HashMap<>();
    List<Thread> threadPool = new ArrayList<>();
    public DrivingControllerTank(HardwareMap hardwareMap)
    {
        RR = new SampleMecanumDrive(hardwareMap);
        mode  = DrivingMode.STOPPED;
        isFollowingTraj.set(false);
        motorMap.clear();
        motorMap.put(MotorName.LF,hardwareMap.get(DcMotorEx.class,"lf"));
        motorMap.put(RF,hardwareMap.get(DcMotorEx.class,"rf"));
        motorMap.put(MotorName.LB,hardwareMap.get(DcMotorEx.class,"lb"));
        motorMap.put(RB,hardwareMap.get(DcMotorEx.class,"rb"));
        PIDreadySignal.set(new HashMap<>());
        PIDdoneSignal.set(new HashMap<>());
    }
    public void shutDown()
    {
        for(Thread tmp : threadPool)
        {
            tmp.interrupt();
        }
        threadPool = new ArrayList<>();
    }
    public void setPos(Pose2d newPos)
    {
        RR.setPoseEstimate(newPos);
    }
    public Pose2d getPos()
    {
        return RR.getPoseEstimate();
    }

    public void doGamepadDriving(Pose2d info)
    {
        RR.setWeightedDrivePower(info);
    }

    public void goForwardSimple(double distance, double power, AtomicReference<Boolean> terminate_signal)
    {
        goForwardSimpleAsync(distance,power,terminate_signal);
        waitForTrajToFinish();
    }
    public void goForwardSimpleAsync(double distance, double power, AtomicReference<Boolean> terminate_signal)
    {
        Log.d("DrivingControllerTank","Init Forwards Simple");
        Thread tmp = new Thread(()-> {
            isFollowingTraj.set(true);
            updateEncoders();
            int deltaTicks = (int) DriveConstantsMain.inchesToEncoderTicks(distance);
            Log.d("DrivingControllerTank","Delta Ticks: " + deltaTicks);
            Map<MotorName, Integer> targetEncoderCounts = new HashMap<>();
            targetEncoderCounts.put(LF, encoderMap.get(LF) + deltaTicks);
            targetEncoderCounts.put(RF, encoderMap.get(RF) + deltaTicks);
            motorMap.get(LF).setPower(power);
            motorMap.get(RF).setPower(power);
            motorMap.get(LB).setPower(power);
            motorMap.get(RB).setPower(power);
            while((encoderMap.get(LF) < targetEncoderCounts.get(LF)) && !terminate_signal.get())
            {
                motorMap.get(LF).setPower(power);
                motorMap.get(RF).setPower(power);
                motorMap.get(LB).setPower(power);
                motorMap.get(RB).setPower(power);
                updateEncoders();
                Log.d("DrivingControllerTank","In progress with delta " + (-encoderMap.get(LF) + targetEncoderCounts.get(LF)));
                RR.update();
            }
            Log.d("DrivingControllerTank","Finished with delta " + Math.abs(encoderMap.get(LF) - targetEncoderCounts.get(LF)));
            motorMap.get(LF).setPower(0);
            motorMap.get(RF).setPower(0);
            motorMap.get(LB).setPower(0);
            motorMap.get(RB).setPower(0);
            isFollowingTraj.set(false);
        });
        threadPool.add(tmp);
        tmp.start();
    }

    public void goBackwardSimpleAsync(double distance, double power, AtomicReference<Boolean> terminate_signal)
    {
        Log.d("DrivingControllerTank","Init Backwards Simple");
        Thread tmp = new Thread(()-> {
            isFollowingTraj.set(true);
            updateEncoders();
            int deltaTicks = (int) DriveConstantsMain.inchesToEncoderTicks(distance);
            Log.d("DrivingControllerTank","Delta Ticks: " + deltaTicks);
            Map<MotorName, Integer> targetEncoderCounts = new HashMap<>();
            targetEncoderCounts.put(LF, encoderMap.get(LF) - deltaTicks);
            targetEncoderCounts.put(RF, encoderMap.get(RF) - deltaTicks);
            motorMap.get(LF).setPower(-power);
            motorMap.get(RF).setPower(-power);
            motorMap.get(LB).setPower(-power);
            motorMap.get(RB).setPower(-power);
            while((encoderMap.get(LF) > targetEncoderCounts.get(LF)) && !terminate_signal.get())
            {
                motorMap.get(LF).setPower(-power);
                motorMap.get(RF).setPower(-power);
                motorMap.get(LB).setPower(-power);
                motorMap.get(RB).setPower(-power);
                updateEncoders();
                Log.d("DrivingControllerTank","In progress with delta " + (encoderMap.get(LF) - targetEncoderCounts.get(LF)));
                RR.update();
            }
            Log.d("DrivingControllerTank","Finished with delta " + Math.abs(encoderMap.get(LF) - targetEncoderCounts.get(LF)));
            motorMap.get(LF).setPower(0);
            motorMap.get(RF).setPower(0);
            motorMap.get(LB).setPower(0);
            motorMap.get(RB).setPower(0);
            isFollowingTraj.set(false);
        });
        threadPool.add(tmp);
        tmp.start();
    }

    public void goBackwardAsync(double distance,double speed)
    {
        Thread tmp = new Thread(()->{
            isFollowingTraj.set(true);
            updateEncoders();
            int deltaTicks = (int) DriveConstantsTank.inchesToEncoderTicks(distance);
            Map<MotorName,Integer> targetEncoderCounts = new HashMap<>();
            targetEncoderCounts.put(LF,encoderMap.get(LF) - deltaTicks);
            targetEncoderCounts.put(RF,encoderMap.get(RF) - deltaTicks);
            double encodersPerSecond = DriveConstantsTank.inchesToEncoderTicks(speed);
            Map<MotorName,Integer[]> encoderTargetLists = new HashMap<>();
            encoderTargetLists.put(LF, createEncoderVsTimeToFollowLINEAR(encodersPerSecond,0.1,encoderMap.get(MotorName.LF),targetEncoderCounts.get(MotorName.LF)));
            encoderTargetLists.put(RF, createEncoderVsTimeToFollowLINEAR(encodersPerSecond,0.1,encoderMap.get(RF),targetEncoderCounts.get(RF)));
            PIDreadySignal.get().put(LF,false);
            PIDdoneSignal.get().put(LF,false);
            PIDreadySignal.get().put(RF,false);
            PIDdoneSignal.get().put(RF,false);
            PIDreadySignal.get().put(MASTER,false);
            new Thread(()->{
                followPIDMotorToEncoderListTank(LF,motorMap.get(LF),motorMap.get(LB),encoderTargetLists.get(LF),0.1,0.5,1,0,0,0.2);
            }).start();
            new Thread(()->{
                followPIDMotorToEncoderListTank(RF,motorMap.get(RF),motorMap.get(RB),encoderTargetLists.get(RF),0.1,0.5,1,0,0,0.2);
            }).start();
            PIDreadySignal.get().put(MASTER,true);
            while(!PIDdoneSignal.get().get(LF) && !PIDdoneSignal.get().get(RF)) {
                PIDreadySignal.get().put(MASTER_LOOP,true);
                while(!(PIDreadySignal.get().get(LF) && PIDreadySignal.get().get(RF)))
                {
                    Sleep.sleep(1);
                }
                PIDreadySignal.get().put(MASTER_LOOP,false);
                PIDreadySignal.get().replace(LF,false);
                PIDreadySignal.get().replace(RF,false);
                RR.update();
            }
            isFollowingTraj.set(false);
        });
        threadPool.add(tmp);
        tmp.start();
    }
    public void goBackward(double distance,double speed)
    {
        goBackwardAsync(distance,speed);
        waitForTrajToFinish();
    }

    public void goForwardAsync(double distance,double speed, AtomicReference<Boolean> terminator)
    {
        Thread tmp = new Thread(()->{
            isFollowingTraj.set(true);
            updateEncoders();
            int deltaTicks = (int) DriveConstantsTank.inchesToEncoderTicks(distance);
            Map<MotorName,Integer> targetEncoderCounts = new HashMap<>();
            targetEncoderCounts.put(MotorName.LF,encoderMap.get(MotorName.LF) + deltaTicks);
            targetEncoderCounts.put(RF,encoderMap.get(RF) + deltaTicks);
            double encodersPerSecond = DriveConstantsTank.inchesToEncoderTicks(speed);
            Map<MotorName,Integer[]> encoderTargetLists = new HashMap<>();
            encoderTargetLists.put(MotorName.LF, createEncoderVsTimeToFollowLINEAR(encodersPerSecond,0.1,encoderMap.get(MotorName.LF),targetEncoderCounts.get(MotorName.LF)));
            encoderTargetLists.put(RF, createEncoderVsTimeToFollowLINEAR(encodersPerSecond,0.1,encoderMap.get(RF),targetEncoderCounts.get(RF)));
            PIDreadySignal.get().put(MotorName.LF,false);
            PIDdoneSignal.get().put(MotorName.LF,false);
            PIDreadySignal.get().put(RF,false);
            PIDdoneSignal.get().put(RF,false);
            PIDreadySignal.get().put(MASTER,false);
            Thread left = new Thread(()->{
                followPIDMotorToEncoderListTank(LF,motorMap.get(LF),motorMap.get(LB),encoderTargetLists.get(LF),0.1,0.5,3,0,0,0.2);
            });
            left.start();
            Thread right = new Thread(()->{
                followPIDMotorToEncoderListTank(RF,motorMap.get(RF),motorMap.get(RB),encoderTargetLists.get(RF),0.1,0.5,3,0,0,0.2);
            });
            right.start();
            PIDreadySignal.get().put(MASTER,true);
            while(!PIDdoneSignal.get().get(LF) && !PIDdoneSignal.get().get(RF) && !terminator.get()) {
                PIDreadySignal.get().put(MASTER_LOOP,true);
                while(!(PIDreadySignal.get().get(LF) && PIDreadySignal.get().get(RF)))
                {
                    Sleep.sleep(1);
                }
                PIDreadySignal.get().put(MASTER_LOOP,false);
                PIDreadySignal.get().replace(LF,false);
                PIDreadySignal.get().replace(RF,false);
                Log.d("DrivingControllerTank", "Cycle complete");
            }
            if(terminator.get())
            {
                left.interrupt();
                right.interrupt();
            }
            isFollowingTraj.set(false);
        });
        threadPool.add(tmp);
        tmp.start();
    }
    public void goForward(double distance,double speed,AtomicReference<Boolean> terminator)
    {
        goForwardAsync(distance,speed,terminator);
        waitForTrajToFinish();
    }

    public void waitForTrajToFinish() {
        while(isFollowingTraj.get())
        {
            Sleep.sleep(10);
        }
    }




    /// MARK: Worker Methods

    private void followPIDMotorToEncoderListTank(MotorName ID, DcMotor motor1, DcMotor motor2, Integer[] encoderTargets, double deltaTimePerSegment, double basePower, double P, double I, double D, double decayFactor)
    {
        int lastError = 0;
        int currentError = 0;
        int totalError = 0;
        while(!PIDreadySignal.get().get(MASTER))
        {
            Sleep.sleep(1);
        }
        for(int index = 0; index < encoderTargets.length; index++)
        {
            while(PIDreadySignal.get().get(ID) || !PIDreadySignal.get().get(MASTER_LOOP))
            {
                Sleep.sleep(1);
            }
            Log.d("DrivingControllerTank", "Start cycle: " + ID);
            currentError = motor1.getCurrentPosition() - encoderTargets[index];
            double powerToApply = floor(cap(basePower + P*currentError + I*totalError + D*(currentError - lastError),1),-1);
            motor1.setPower(powerToApply);
            motor2.setPower(powerToApply);
            Log.d("DrivingControllerTank", "ID: " + ID + " power sent: " + powerToApply);
            lastError = currentError;
            totalError*=decayFactor;
            totalError += currentError;
            PIDreadySignal.get().replace(ID,true);
            Log.d("DrivingControllerTank", "Ready and waiting... " + ID);
        }
        motor1.setPower(0);
        motor2.setPower(0);
        PIDdoneSignal.get().replace(ID,true);
    }

    private Integer[] createEncoderVsTimeToFollowLINEAR(double encodersPerSecond, double deltaTime, int iniEncoder, int finEncoder)
    {
        int encoderDelta = (finEncoder - iniEncoder);
        double totalTime = encoderDelta/encodersPerSecond;
        int totalSegments = (int) (totalTime / deltaTime)+1;
        int encoderSegmentDelta = encoderDelta / totalSegments;
        Integer[] toFollow = new Integer[totalSegments];
        for(int index = 0; index < toFollow.length; index++)
        {
            toFollow[index] = iniEncoder + index*encoderSegmentDelta;
        }
        return toFollow;

    }

    private double cap(double quantity, double cap)
    {
        if(quantity > cap)
        {
            return cap;
        }
        else return quantity;
    }
    private double floor(double quantity, double floor)
    {
        if(quantity < floor)
        {
            return floor;
        }
        else return quantity;
    }
    private void updateEncoders()
    {
        encoderMap.clear();
        encoderMap.put(MotorName.LF,motorMap.get(MotorName.LF).getCurrentPosition());
        encoderMap.put(RF,motorMap.get(RF).getCurrentPosition());
        encoderMap.put(RB,motorMap.get(RB).getCurrentPosition());
        encoderMap.put(MotorName.LB,motorMap.get(MotorName.LB).getCurrentPosition());
    }

    public void stopMotors() {
        for(DcMotor motor : motorMap.values())
        {
            motor.setPower(0);
        }
    }
}
