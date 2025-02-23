package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Settings.AlgaeArmSettings;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.utility.MotionProfile2d;
import frc.robot.utility.Vector2d;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.Functions;


public class AlgaeArmSystem extends SubsystemBase {

    /*                                     -   +                                 From Lower Joint (default)
        +                          - - - - - 0 + + + + + + + + + + + + + + + + + + + + + + + + +  
        |                  %/@(                                                                 |
        |                 &@%%                                                                  |
        |                 ,,,& & @                                                              |
        |                 ,,@&//%%                                                              |
        |                ,,, @%# ¯}                       #,(                                   |
        |                ,,  @%%                        @*O,&\____________..±±_____%#//]        |
        |               ,,, @/@                        #,,,%//____________.¶#&@@==== §          |
        |               ,,@&  /                      %,,,&                  &|     %#//]        |
        |              @,,  / (  &&                @%,,(                                        |
        |              ,,(((/(@  (&               &,,,#                                         |
        |             ,,, (((@   ,,             #,,,@                                           |
        |             ,,, @@C    ,,%          .@,,@/                                            |
        |            @,,&((@(    %%@         @,,,%               From     Lower J     Robot     | +
        |            ,,,(/@@     %%@       &#O,@                           180*         90      0
        |           @,,     ,.....,&....   &/&(/{}]           Upper J:  270*+ 90*   180 + 0     | -
        |          @#,,@    ,....  &...@    ,,, ¯¯                          0*         270      |
        |          .&((L_   ,....  @..%/#@&@@@@                                            
        |         /(((*(((\ &##%(  @(##&@@@#@@@(%%                         180          90               
        |       /((/@%&%@((\@,.(*  @(.(,&&%&%#%%%##@          Lower J:  270 + 90    180 + 0                    
        |  [¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯C)¯C)¯|¯¯¯]                   0          270      
       +|  [__________________________________/__/__1___]     * - only the case when the lower J is 180
        0       '¯'               0              '¯'          
       -| - - - - - - - - - - - - ^ + + + + + + + + + + + + + + + + + + + + + + 
        |                     From Robot
     */

    public boolean RightBias; // Default is based on starting config. This determines which side the middle joint will always try to stay on 
    // relative to a line between the lower joint and the target. This is because there are almost always two solutions to the kinematics

    private double CurrentL1Angle, CurrentL2Angle; // radians
    private Vector2d MovingTarget; // in inches and (0, 0) is at the lower joint
    public Vector2d Target;

    private DoubleSupplier L1Encoder, L2Encoder;

    private MotionProfile2d motionProfile;

    private double L1Offset, L2Offset; 

    private ElapsedTime frameTimer;
    private double frameTime = 0.01;

    private TalonFX bottomPivot;
    private TalonFX topPivot;
   


    /**
     * The 0 angles for each joint is when the arm is perfectly inside the segment it is attached to 
     * Basically the arm is pointing straight up on the robot when the lower joint and upper joint are pi (180 degrees).
     */
    public AlgaeArmSystem(double firstSegmentStartAngle, double secondSegmentStartAngle) {
        L1Offset = firstSegmentStartAngle;
        L2Offset = secondSegmentStartAngle;

        if (Functions.normalizeAngle(secondSegmentStartAngle) > Math.PI) RightBias = true;
        else RightBias = false;

        // initialize motors
        bottomPivot = new TalonFX(MotorConstants.alageBottomPivotID);
        topPivot = new TalonFX(MotorConstants.alageTopPivotID);
        // initialize PIDs
        L1Encoder = () -> bottomPivot.getPosition().getValueAsDouble() + L1Offset; //I think this should get the currect position but will need testing
        L2Encoder = () -> bottomPivot.getPosition().getValueAsDouble() + L2Offset; 

        CurrentL1Angle = L1Encoder.getAsDouble();
        CurrentL2Angle = L2Encoder.getAsDouble();

        // Get start position
        MovingTarget = getCurrentPoint();
        Target = MovingTarget;

        motionProfile = new MotionProfile2d(Target, AlgaeArmSettings.maxAcceleration, AlgaeArmSettings.maxDeceleration, AlgaeArmSettings.maxSpeed);

        frameTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }




    @Override
    public void periodic() { // Main run method
        frameTime = frameTimer.time();
        frameTimer.reset();

        //TODO: We only need this if we can and want to tune these variables while driving
        motionProfile.updateSettings(AlgaeArmSettings.maxAcceleration, AlgaeArmSettings.maxDeceleration, AlgaeArmSettings.maxSpeed);

        // Update encoders
        CurrentL1Angle = L1Encoder.getAsDouble();
        CurrentL2Angle = L2Encoder.getAsDouble();

        // Make sure final target is reachable
        if (Target.mag() > AlgaeArmConstants.LowerSegmentLength + AlgaeArmConstants.UpperSegmentLength) {
            Target.setMag(AlgaeArmConstants.LowerSegmentLength + AlgaeArmConstants.UpperSegmentLength);
        } else if (Target.mag() < Math.abs(AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength)) {
            Target.setMag(Math.abs(AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength));
        }

        Target.x = Functions.minMaxValue(AlgaeArmSettings.maxDistanceInX, AlgaeArmSettings.maxDistanceOutX, Target.x);
        if (Target.y < AlgaeArmSettings.maxDistanceDownY) Target.y = AlgaeArmSettings.maxDistanceDownY;

        // Get moving target from motion profile
        motionProfile.setFinalTarget(Target);
        MovingTarget = motionProfile.update();

        // Make sure motion profile target is within reachable positions and won't clip into things
        if (MovingTarget.mag() > AlgaeArmConstants.LowerSegmentLength + AlgaeArmConstants.UpperSegmentLength) {
            MovingTarget.setMag(AlgaeArmConstants.LowerSegmentLength + AlgaeArmConstants.UpperSegmentLength);
        } else if (MovingTarget.mag() < Math.abs(AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength)) {
            MovingTarget.setMag(Math.abs(AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength));
        }

        MovingTarget.x = Functions.minMaxValue(AlgaeArmSettings.maxDistanceInX, AlgaeArmSettings.maxDistanceOutX, MovingTarget.x);
        if (MovingTarget.y < AlgaeArmSettings.maxDistanceDownY) MovingTarget.y = AlgaeArmSettings.maxDistanceDownY;

        double TargetL1Angle = getTargetLowerAngle();
        double TargetL2Angle = getTargetUpperAngle();
        TargetL1Angle = Functions.minMaxValue(AlgaeArmSettings.minAngleLowerJoint, AlgaeArmSettings.maxAngleLowerJoint, TargetL1Angle);
        TargetL2Angle = Functions.minMaxValue(AlgaeArmSettings.minAngleUpperJoint, AlgaeArmSettings.maxAngleUpperJoint, TargetL2Angle);
        setTargetAngles(TargetL1Angle, TargetL2Angle);

        // Use inverse kinematics to get new target joint angles and pass that to the PIDs
        double L1MotorPower = AlgaeArmSettings.LowerJointPID.calculate(CurrentL1Angle, getTargetLowerAngle());
        double L2MotorPower = AlgaeArmSettings.UpperJointPID.calculate(CurrentL2Angle, getTargetUpperAngle());
        
        // Add the gravity/acceleration compensation power


        // Give power to motors
        bottomPivot.setVoltage(toVoltage(L1MotorPower));
        topPivot.setVoltage(toVoltage(L2MotorPower));

        // Telemetry
        //SmartDashboard.putString("Alge Encoder 1", ""+L1Encoder);
        //SmartDashboard.putString("Alge Encoder 1", ""+L2Encoder);

    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    private double toVoltage(double power){
        if(Math.abs(power)<AlgaeArmSettings.voltageTolerance)
        return 0;
        
        power = (AlgaeArmConstants.maxVoltage-AlgaeArmConstants.minVoltage)*power + (power/Math.abs(power))*AlgaeArmConstants.minVoltage; //adds the min value + the range between the max and min voltage to get a number between the min and max proporitnal to the power
        return power;
        
    }



    // KINEMATIC AND INVERSE KINEMATIC METHODS - sorry for making 623 different methods
    
    /**
     * Gets the current height and distance forward of the end of the arm based from the lower joint.
     */
    public Vector2d getCurrentPoint() {
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos((CurrentL1Angle - Math.PI/2)) + AlgaeArmConstants.UpperSegmentLength * Math.cos(Math.PI - CurrentL2Angle - (CurrentL1Angle - Math.PI/2)),
            AlgaeArmConstants.LowerSegmentLength * Math.sin((CurrentL1Angle - Math.PI/2)) + AlgaeArmConstants.UpperSegmentLength * Math.sin(Math.PI + CurrentL2Angle + (CurrentL1Angle - Math.PI/2))
        );
    }
    /**
     * Gets the current height and distance forward of the end of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getCurrentPointFromRobot() {
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos((CurrentL1Angle - Math.PI/2)) + AlgaeArmConstants.UpperSegmentLength * Math.cos(Math.PI - CurrentL2Angle - (CurrentL1Angle - Math.PI/2)) + AlgaeArmConstants.LowerJointForward,
            AlgaeArmConstants.LowerSegmentLength * Math.sin((CurrentL1Angle - Math.PI/2)) + AlgaeArmConstants.UpperSegmentLength * Math.sin(Math.PI + CurrentL2Angle + (CurrentL1Angle - Math.PI/2)) + AlgaeArmConstants.LowerJointHeight
        );
    }

    /**
     * Gets the final target height and distance forward of the end of the arm based from the lower joint.
     */
    public Vector2d getTarget() {
        return Target;
    }
    /**
     * Gets the current motion profile target height and distance forward of the end of the arm based from the lower joint.
     */
    public Vector2d getMovingTarget() {
        return MovingTarget;
    }
    /**
     * Gets the target height and distance forward of the end of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getTargetFromRobot() {
        return Target.plus(new Vector2d(AlgaeArmConstants.LowerJointForward, AlgaeArmConstants.LowerJointHeight));
    }
    /**
     * Gets the current motion profile target height and distance forward of the end of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getMovingTargetFromRobot() {
        return MovingTarget.plus(new Vector2d(AlgaeArmConstants.LowerJointForward, AlgaeArmConstants.LowerJointHeight));
    }

    /**
     * Gets the current height and distance forward of the second joint of the arm based from the lower joint.
     */
    public Vector2d getCurrentMiddlePoint() {
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos((CurrentL1Angle - Math.PI/2)),
            AlgaeArmConstants.LowerSegmentLength * Math.sin((CurrentL1Angle - Math.PI/2))
        );
    }
    /**
     * Gets the current height and distance forward of the second joint of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getCurrentMiddlePointFromRobot() {
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos((CurrentL1Angle - Math.PI/2)) + AlgaeArmConstants.LowerJointForward,
            AlgaeArmConstants.LowerSegmentLength * Math.sin((CurrentL1Angle - Math.PI/2)) + AlgaeArmConstants.LowerJointHeight
        );
    }
    /**
     * Gets the target height and distance forward of the second joint of the arm based from the lower joint.
     */
    public Vector2d getTargetMiddlePoint() {
        double L1Angle = getTargetLowerAngle() - Math.PI/2;
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(L1Angle),
            AlgaeArmConstants.LowerSegmentLength * Math.sin(L1Angle)
        );
    }
    /**
     * Gets the target height and distance forward of the second joint of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getTargetMiddlePointFromRobot() {
        double L1Angle = getTargetLowerAngle() - Math.PI/2;
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(L1Angle) + AlgaeArmConstants.LowerJointForward,
            AlgaeArmConstants.LowerSegmentLength * Math.sin(L1Angle) + AlgaeArmConstants.LowerJointHeight
        );
    }
    /**
     * Gets the target height and distance forward of the second joint of the arm based from the lower joint.
     */
    public Vector2d getMovingTargetMiddlePoint() {
        double L1Angle = getMovingTargetLowerAngle() - Math.PI/2;
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(L1Angle),
            AlgaeArmConstants.LowerSegmentLength * Math.sin(L1Angle)
        );
    }
    /**
     * Gets the target height and distance forward of the second joint of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getMovingTargetMiddlePointFromRobot() {
        double L1Angle = getMovingTargetLowerAngle() - Math.PI/2;
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(L1Angle) + AlgaeArmConstants.LowerJointForward,
            AlgaeArmConstants.LowerSegmentLength * Math.sin(L1Angle) + AlgaeArmConstants.LowerJointHeight
        );
    }

    public double getCurrentLowerAngle() { return Functions.normalizeAngle(CurrentL1Angle); }
    public double getCurrentLowerAngleFromRobot() { return Functions.normalizeAngle(CurrentL1Angle - Math.PI/2); }

    public double getCurrentUpperAngle() { return Functions.normalizeAngle(CurrentL2Angle); }
    public double getCurrentUpperAngleFromRobot() { return Functions.normalizeAngle((Math.PI - CurrentL2Angle) - getCurrentLowerAngleFromRobot()); }

    public double getTargetLowerAngle() { return Functions.normalizeAngle(Target.angle() + Math.asin((AlgaeArmConstants.UpperSegmentLength * Math.sin(getTargetUpperAngle())) / Target.mag()) + Math.PI/2); }
    public double getTargetLowerAngleFromRobot() { return Functions.normalizeAngle(getTargetLowerAngle() - Math.PI/2); }

    public double getMovingTargetLowerAngle() { return Functions.normalizeAngle(MovingTarget.angle() + Math.asin((AlgaeArmConstants.UpperSegmentLength * Math.sin(getMovingTargetUpperAngle())) / MovingTarget.mag()) + Math.PI/2); }
    public double getMovingTargetLowerAngleFromRobot() { return Functions.normalizeAngle(getMovingTargetLowerAngle() - Math.PI/2); }

    public double getTargetUpperAngle() {
        double result = Math.acos(((Target.x * Target.x + Target.y * Target.y) - AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength * AlgaeArmConstants.UpperSegmentLength) 
            / (-2 * AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.UpperSegmentLength));
        if (RightBias) return Functions.normalizeAngle(-1 * result);
        else return Functions.normalizeAngle(result);
    }
    public double getTargetUpperAngleFromRobot() { return Functions.normalizeAngle((Math.PI - getTargetUpperAngle()) - getTargetLowerAngleFromRobot()); }

    public double getMovingTargetUpperAngle() {
        double result = Math.acos(((MovingTarget.x * MovingTarget.x + MovingTarget.y * MovingTarget.y) - AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength * AlgaeArmConstants.UpperSegmentLength) 
            / (-2 * AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.UpperSegmentLength));
        if (RightBias) return Functions.normalizeAngle(-1 * result);
        else return Functions.normalizeAngle(result);
    }
    public double getMovingTargetUpperAngleFromRobot() { return Functions.normalizeAngle((Math.PI - getMovingTargetUpperAngle()) - getMovingTargetLowerAngleFromRobot()); }




    // SET TARGET METHODS
    
    /**
     * Sets the target height and distance forward of the end of the arm based from the lower joint.
     */
    public void setTarget(Vector2d target) {
        Target = target;
    }
    /**
     * an angle of pi is straight up 
     * Keeps the upper target angle at the same angle relative to the robot
     */
    public void setTargetLowerAngle(double lowerTargetAngle) { setTargetAnglesFromRobot(lowerTargetAngle - Math.PI/2, getTargetUpperAngleFromRobot()); }
    /**
     * an angle of pi is straight relative to the lower arm 
     * Keeps the lower target angle at the same angle relative to the robot
     */
    public void setTargetUpperAngle(double upperTargetAngle) { setTargetAngles(getTargetLowerAngle(), upperTargetAngle); }
    /**
     * Sets the target height and distance forward of the end of the arm based from the center of the robot frame and the floor
     */
    public void setTargetPointFromRobot(Vector2d target) { Target = target.minus(new Vector2d(AlgaeArmConstants.LowerJointForward, AlgaeArmConstants.LowerJointHeight)); }
    /**
     * pi/2 is straight up
     * Keeps the upper target angle at the same angle relative to the robot
     */
    public void setTargetLowerAngleFromRobot(double lowerTargetAngle) { setTargetAnglesFromRobot(lowerTargetAngle, getTargetUpperAngleFromRobot()); }
    /**
     * pi is straight up for lower arm, pi is straight out relative to lower arm angle for the upper segment
     */
    public void setTargetUpperAngleFromRobot(double upperTargetAngle) { setTargetAnglesFromRobot(getCurrentLowerAngleFromRobot(), upperTargetAngle); }
    /**
     * An angle of pi is straight up for lower Arm while an angle of pi is straight relative to the lower arm for the upper segment
     */
    public void setTargetAngles(double lowerTargetAngle, double upperTargetAngle) {
        lowerTargetAngle = Functions.minMaxValue(AlgaeArmSettings.minAngleLowerJoint, AlgaeArmSettings.maxAngleLowerJoint, lowerTargetAngle);
        upperTargetAngle = Functions.minMaxValue(AlgaeArmSettings.minAngleUpperJoint, AlgaeArmSettings.maxAngleUpperJoint, upperTargetAngle);
        Target = new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos((lowerTargetAngle - Math.PI/2)) + AlgaeArmConstants.UpperSegmentLength * Math.cos(Math.PI - upperTargetAngle - (lowerTargetAngle - Math.PI/2)),
            AlgaeArmConstants.LowerSegmentLength * Math.sin((lowerTargetAngle - Math.PI/2)) + AlgaeArmConstants.UpperSegmentLength * Math.sin(Math.PI + upperTargetAngle + (lowerTargetAngle - Math.PI/2))
        );
    }
    /**
     * An angle of pi/2 is straight up for both segments
     */
    public void setTargetAnglesFromRobot(double lowerTargetAngle, double upperTargetAngle) {
        double FromRobotLowerAngle = Functions.normalizeAngle(lowerTargetAngle + Math.PI/2);
        double FromRobotUpperAngle = Functions.normalizeAngle((Math.PI + upperTargetAngle) - FromRobotLowerAngle + Math.PI/2);
        FromRobotLowerAngle = Functions.minMaxValue(AlgaeArmSettings.minAngleLowerJoint, AlgaeArmSettings.maxAngleLowerJoint, FromRobotLowerAngle);
        FromRobotUpperAngle = Functions.minMaxValue(AlgaeArmSettings.minAngleUpperJoint, AlgaeArmSettings.maxAngleUpperJoint, FromRobotUpperAngle);
        Target = new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos((FromRobotLowerAngle - Math.PI/2)) + AlgaeArmConstants.UpperSegmentLength * Math.cos(Math.PI - FromRobotUpperAngle - (FromRobotLowerAngle - Math.PI/2)),
            AlgaeArmConstants.LowerSegmentLength * Math.sin((FromRobotLowerAngle - Math.PI/2)) + AlgaeArmConstants.UpperSegmentLength * Math.sin(Math.PI + FromRobotUpperAngle + (FromRobotLowerAngle - Math.PI/2))
        );
    }




    // OTHER METHODS

    /**
     * Moves the target by the velocity inputted
     * @param x - in/s
     * @param y - in/s
     */
    public void setTargetMoveSpeeds(double x, double y) {
        Target = new Vector2d(MovingTarget.x + x * frameTime, MovingTarget.y + y * frameTime);
    }

    public boolean isCloseToTarget() {
        return Target.distFrom(getCurrentPoint()) < AlgaeArmSettings.PositionTolerance;
    }

    public boolean isCloseToTarget(Vector2d target) {
        return target.distFrom(getCurrentPoint()) < AlgaeArmSettings.PositionTolerance;
    }

    public void setRightBias(boolean rightBias) { RightBias = rightBias; }

    public boolean getRightBias() { return RightBias; }

    public void setTargetMaxInDirection(double angle) {
        Target.setMag(AlgaeArmConstants.LowerSegmentLength + AlgaeArmConstants.UpperSegmentLength);
        Target.setAngle(angle);
    }

}
