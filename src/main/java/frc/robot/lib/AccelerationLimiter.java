package frc.robot.lib;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

public class AccelerationLimiter {

    private final double maxAcceleration;
    private final double maxJerk;

    private double previousVelocity;
    private double previousAccelerationLimit;

    private double previousTime;

    public AccelerationLimiter(double maxAcceleration, double maxJerk) {

        this.maxAcceleration = maxAcceleration;
        this.maxJerk = maxJerk;

        previousVelocity = 0;
        previousAccelerationLimit = 0;

        previousTime = -1;
        previousTime = MathSharedStore.getTimestamp();
    }

    public double calculate(double input)
    {
        double timeDelta = MathSharedStore.getTimestamp() - previousTime;

        double deltaVelocity = input-previousVelocity;

        double maxAccelerationLimited = previousAccelerationLimit + MathUtil.clamp(
                deltaVelocity - previousAccelerationLimit,
                -maxJerk * timeDelta,
                maxJerk * timeDelta
        );

        maxAccelerationLimited = MathUtil.clamp(
                maxAccelerationLimited,
                -maxAcceleration,
                maxAcceleration
        );

        double velocity = previousVelocity + MathUtil.clamp(
                input - previousVelocity,
                -maxAccelerationLimited * timeDelta,
                maxAccelerationLimited * timeDelta
        );

        previousTime = MathSharedStore.getTimestamp();
        previousAccelerationLimit = maxAccelerationLimited;
        previousVelocity = velocity;

        return velocity;
    }
}
