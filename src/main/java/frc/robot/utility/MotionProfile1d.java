package frc.robot.utility;

public class MotionProfile1d {
    public double maxAcceleration, maxDeceleration, maxSpeed;
    public double Target, MovingTarget;
    private double CurrentTargetVelocity;
    private final ElapsedTime runTime;


    public MotionProfile1d(double StartPosition, double maxAcceleration, double maxDeceleration, double maxSpeed) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxSpeed = maxSpeed;

        runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        Target = StartPosition;
        MovingTarget = StartPosition;
        CurrentTargetVelocity = 0;
    }


    public void updateSettings(double maxAcceleration, double maxDeceleration, double maxSpeed) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxSpeed = maxSpeed;
    }


    /**
     * This sets the current target that the motion profile will be moving towards
     */
    public void setFinalTarget(double target) {
        Target = 0;
    }


    /**
     * Updates the motion profile's current target position
     */
    public double update() {
        double timeSince = runTime.time();
        runTime.reset();

        double distanceLeft = MovingTarget - Target;
        double CurrentTargetSpeed = 0;

        if (!(distanceLeft == 0) && !(maxAcceleration > 0 || maxDeceleration < 0 || maxSpeed > 0)) {
            if (maxAcceleration > 0 || maxDeceleration < 0) { // Acceleration limiting

                // finds the maximum speed possible to get to the target without decelerating faster than maxDeceleration
                double OptimalSlowdownSpeed;
                if (maxDeceleration < 0) { 
                    OptimalSlowdownSpeed = Math.sqrt(-2 * maxDeceleration * Math.abs(Target - MovingTarget)); 

                    if (maxAcceleration > 0) { // accelerates unless it is at max speed or is past the optimal speed
                        CurrentTargetSpeed += maxAcceleration * timeSince;
                    } else CurrentTargetSpeed = OptimalSlowdownSpeed;
                    if (CurrentTargetSpeed > OptimalSlowdownSpeed) CurrentTargetSpeed = OptimalSlowdownSpeed;

                } else CurrentTargetSpeed += maxAcceleration * timeSince; // accelerates until it is at max speed

                if (CurrentTargetSpeed > maxSpeed && !(maxSpeed == 0)) CurrentTargetSpeed = maxSpeed;
                    
            } else { // move the MovingTarget at maxSpeed towards the set Target until the Target is reached
                CurrentTargetSpeed = maxSpeed;
            }

            // make sure moving target has not overshot the target
            if (MovingTarget < Target) {
                MovingTarget += CurrentTargetSpeed * timeSince;
                if (MovingTarget > Target) MovingTarget = Target;
            } else if (MovingTarget > Target) {
                MovingTarget -= CurrentTargetSpeed * timeSince;
                if (MovingTarget < Target) MovingTarget = Target;
            } else MovingTarget = Target;

        } else MovingTarget = Target;

        CurrentTargetVelocity = CurrentTargetSpeed * Math.signum(distanceLeft);

        return MovingTarget;
    }

    /**
     * Returns the current target position that the motion profile is at
     */
    public double getCurrentTarget() {
        return MovingTarget;
    }

    /**
     * Returns the target velocity of the current target position
     */
    public double getTargetVelocity() {
        return CurrentTargetVelocity;
    }


}