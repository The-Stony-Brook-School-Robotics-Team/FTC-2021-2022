package org.firstinspires.ftc.teamcode.common.newAutonomous;


import com.acmerobotics.roadrunner.util.NanoClock;

public class AutonomousTimer {
    private static final double PICK_UP_TO_DEPOSIT_TIME_CHECK_DURATION = 4.0;
    private static final double DEPOSIT_TO_PICK_UP_TIME_CHECK_DURATION = 7.0;   //Includes time for parking

    private static double endTime_s;
    private static boolean canContinue = true;

    public static volatile CurrentState currentState = CurrentState.DepositToPickUp;

    public static void startTimer() {
        canContinue = true;
        endTime_s = NanoClock.system().seconds() + 30;
    }

    public static boolean canContinue() {
        return canContinue;
    }

    public static boolean canContinue(CurrentState currentState) {
        if (!canContinue)
            return false;
        switch (currentState) {
            case PickUpToDeposit:
                canContinue = NanoClock.system().seconds() < endTime_s - PICK_UP_TO_DEPOSIT_TIME_CHECK_DURATION;
                AutonomousTimer.currentState = CurrentState.PickUpToDeposit;
                break;
            case DepositToPickUp:
                AutonomousTimer.currentState = CurrentState.DepositToPickUp;
                canContinue = NanoClock.system().seconds() < endTime_s - DEPOSIT_TO_PICK_UP_TIME_CHECK_DURATION;
        }
        return canContinue;
    }

    public static void setEndTime_s(double endTime_s) {
        AutonomousTimer.endTime_s = endTime_s;
    }

    public enum CurrentState {
        PickUpToDeposit,
        DepositToPickUp
    }
}
