package org.firstinspires.ftc.teamcode.common.newAutonomous;


import com.acmerobotics.roadrunner.util.NanoClock;

public class AutonomousTimer {
    private static final double PICK_UP_TO_DEPOSIT_TIME_CHECK_DURATION = 4.0;
    private static final double PICK_UP_SECONDARY_TO_DEPOSIT_CHECK_DURATION = 4.0;
    private static final double DEPOSIT_TO_PICK_UP_TIME_CHECK_DURATION = 8.0;   //Includes time for parking

    public static double endTime_s;

    private static boolean canContinue = true;

    public static void startTimer() {
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
                break;
            case PickUpSecondaryToDeposit:
                canContinue = NanoClock.system().seconds() < endTime_s - PICK_UP_SECONDARY_TO_DEPOSIT_CHECK_DURATION;
                break;
            case DepositToPickUp:
                canContinue = NanoClock.system().seconds() < endTime_s - DEPOSIT_TO_PICK_UP_TIME_CHECK_DURATION;
        }
        return canContinue;
    }

    public static void setEndTime_s(double endTime_s) {
        AutonomousTimer.endTime_s = endTime_s;
    }

    public enum CurrentState {
        PickUpToDeposit,
        PickUpSecondaryToDeposit,
        DepositToPickUp
    }
}
