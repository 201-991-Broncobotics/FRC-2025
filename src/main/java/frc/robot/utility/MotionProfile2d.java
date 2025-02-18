package frc.robot.utility;

public class MotionProfile2d {


    public double maxAcceleration, maxDeceleration, maxSpeed;
    public Vector2d Target, MovingTarget;
    private Vector2d CurrentTargetVelocity;
    private final ElapsedTime runTime;


    public MotionProfile2d(Vector2d StartPosition, double maxAcceleration, double maxDeceleration, double maxSpeed) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxSpeed = maxSpeed;

        runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        Target = StartPosition;
        MovingTarget = StartPosition;
        CurrentTargetVelocity = new Vector2d();
    }


    public void updateSettings(double maxAcceleration, double maxDeceleration, double maxSpeed) {
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.maxSpeed = maxSpeed;
    }


    /**
     * This sets the current target that the motion profile will be moving towards
     */
    public void setFinalTarget(Vector2d target) {
        Target = target;
    }


    /**
     * Updates the motion profile's current target position
     */
    public Vector2d update() {

        double timeSince = runTime.time();
        runTime.reset();


        Vector2d distanceLeft = MovingTarget.minus(Target);

        if (maxAcceleration > 0 || maxDeceleration < 0) { // Acceleration limiting
            double OptimalSlowdownSpeed;


            if (!(distanceLeft.mag() == 0)) {
                if (maxDeceleration < 0) OptimalSlowdownSpeed = Math.sqrt(-2 * maxDeceleration * Target.distFrom(MovingTarget)); // finds the maximum speed possible to get to the target without decelerating faster than max
                else OptimalSlowdownSpeed = maxSpeed;
                if (OptimalSlowdownSpeed > maxSpeed) OptimalSlowdownSpeed = maxSpeed;


                if (maxAcceleration > 0) { // accelerates until is at max speed or is at optimal speed
                    CurrentTargetVelocity = CurrentTargetVelocity.plus((new Vector2d()).withMag(maxAcceleration).withAngle(distanceLeft.angle()));
                } else CurrentTargetVelocity.setMag(OptimalSlowdownSpeed);
                if (CurrentTargetVelocity.mag() > OptimalSlowdownSpeed) CurrentTargetVelocity.setMag(OptimalSlowdownSpeed);

            }
        } else { // move the movingTargetPosition at maxSpeed towards the set targetPosition until the targetPosition is reached
            CurrentTargetVelocity = (new Vector2d()).withMag(maxSpeed).withAngle(distanceLeft.angle());
        }

        Vector2d newMovingTarget = MovingTarget.plus(CurrentTargetVelocity.times(timeSince)); // move moving target by current velocity

        // Sorry to hurt your brain with math but this finds if the new moving
        // target has travelled into the other half of the coord grid that the target is on based on the previous moving target.
        // Though I probably could have just figured out whether the angle to the target has changed by more than 90 degrees
        boolean overshotTarget;
        if (MovingTarget.y - Target.y == 0) overshotTarget = newMovingTarget.x - Target.x < 0;
        else overshotTarget = (((-1 * (MovingTarget.x - Target.x)) / (MovingTarget.y - Target.y)) * (newMovingTarget.x - Target.x) + Target.y - newMovingTarget.y) * (Math.signum(Target.y - MovingTarget.y)) < 0;

        if (overshotTarget) MovingTarget = Target; // set to final target if overshot
        else MovingTarget = newMovingTarget;

        return MovingTarget;
    }

    /**
     * Returns the current target position that the motion profile is at
     */
    public Vector2d getCurrentTarget() {
        return MovingTarget;
    }


}