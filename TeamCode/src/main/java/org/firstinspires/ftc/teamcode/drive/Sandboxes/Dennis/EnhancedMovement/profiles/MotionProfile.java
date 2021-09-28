package org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.profiles;

import org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.annotations.Testing;
import org.jetbrains.annotations.NotNull;
import org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.enums.MotionProfileType;

// TODO: Adjust Variables To More Reasonable Limiters
@Testing
/**
 * Motion profile class that lets us define certain motor characteristics
 */
public class MotionProfile {


    public double MAX_MOTOR_VELO;
    public double MIN_MOTOR_VELO;
    public double MOTOR_VELO_MULTIPLIER;

    public MotionProfile(@NotNull double MAX_MOTOR_VELO, @NotNull double MIN_MOTOR_VELO, @NotNull double MOTOR_VELO_MULTIPLIER)
    {
        this.MAX_MOTOR_VELO = MAX_MOTOR_VELO;
        this.MIN_MOTOR_VELO = MIN_MOTOR_VELO;
        this.MOTOR_VELO_MULTIPLIER = MOTOR_VELO_MULTIPLIER;
    }

    public MotionProfile(@NotNull boolean USE_DEFAULT_PROFILE, @NotNull MotionProfileType motorProfile)
    {
        if(USE_DEFAULT_PROFILE) { load_default_profile(motorProfile);}
    }

    /**
     * lets us load default parameters that we can tweak that will help smooth motor movement with FTCLIB
     * @param motorProfile is a new motion profile that we define in MotionProfile
     */
    private void load_default_profile(@NotNull MotionProfileType motorProfile)
    {
        switch(motorProfile)
        {
            case FAST:
                MAX_MOTOR_VELO = 1;
                MIN_MOTOR_VELO = 0.5;
                MOTOR_VELO_MULTIPLIER = 2;
            case SLOW:
                MAX_MOTOR_VELO = 0.4;
                MIN_MOTOR_VELO = 0.2;
                MOTOR_VELO_MULTIPLIER = 1.2;
            case PRECISE:
                MAX_MOTOR_VELO = 0.6;
                MIN_MOTOR_VELO = 0.5;
                MOTOR_VELO_MULTIPLIER = 1.1;
        }
    }

}
