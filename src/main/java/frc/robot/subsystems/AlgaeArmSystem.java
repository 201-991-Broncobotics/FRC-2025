package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import frc.robot.Settings.AlgaeArmSettings;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.utility.MotionProfile2d;
import frc.robot.utility.Vector2d;
import frc.robot.utility.Functions;


public class AlgaeArmSystem extends SubsystemBase {
    // `¡™£¢∞§¶•ªº–≠“‘«…æ≤≥÷`⁄€‹›ﬁﬂ‡°·‚—±”’»ÚÆ¯˘¿      
    /*                                     -   +                                 From Lower Joint
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
        |            @,,&((@(    %%@         @,,,%                                              | +
        |            ,,,(/@@     %%@       &#O,@                                                0
        |           @,,     ,.....,&....   &/&(/{}]                                             | -
        |          @#,,@    ,....  &...@    ,,, ¯¯                                              |
        |          .&((L_   ,....  @..%/#@&@@@@                                            
        |         /(((*(((\ &##%(  @(##&@@@#@@@(%%                                         
        |       /((/@%&%@((\@,.(*  @(.(,&&%&%#%%%##@                                 
        |  [¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯C)¯C)¯|¯¯¯]                                   
       +|  [__________________________________/__/__1___]     
        0       '¯'               0              '¯'
       -| - - - - - - - - - - - - ^ + + + + + + + + + + + + + + + + + + + + + + 
        |                     From Robot
     */

    public boolean RightBias; // Default is based on starting config. This determines which side the middle joint will always try to stay on 
    // relative to a line between the lower joint and the target. This is because there are almost always two solutions to the kinematics

    private double CurrentL1Angle, CurrentL2Angle; // radians
    private Vector2d Target; // in inches and (0, 0) is at the lower joint

    private DoubleSupplier L1Encoder, L2Encoder;

    private MotionProfile2d motionProfile;

    private double L1Offset, L2Offset; 


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

        // initialize PIDs

        L1Encoder = () -> 0 + L1Offset; //TODO: add get Encoder position and conversion to radians
        L2Encoder = () -> 0 + L2Offset; //TODO: add get Encoder position and conversion to radians

        CurrentL1Angle = L1Encoder.getAsDouble();
        CurrentL2Angle = L2Encoder.getAsDouble();

        // Get start position
        Target = new Vector2d(); // TODO: set this to something before running

        motionProfile = new MotionProfile2d(Target, AlgaeArmSettings.maxAcceleration, AlgaeArmSettings.maxDeceleration, AlgaeArmSettings.maxSpeed);
    }


    @Override
    public void periodic() { // Main run method

        //TODO: We only need this if we can and want to tune these variables while driving
        motionProfile.updateSettings(AlgaeArmSettings.maxAcceleration, AlgaeArmSettings.maxDeceleration, AlgaeArmSettings.maxSpeed);

        // Update encoders
        CurrentL1Angle = L1Encoder.getAsDouble();
        CurrentL2Angle = L2Encoder.getAsDouble();

        // Get moving target from motion profile
        Target = motionProfile.update();

        // Make sure target is within reachable positions and won't clip into things

        // Use inverse kinematics to get new target joint angles and pass that to the PIDs
        double L1MotorPower = AlgaeArmSettings.LowerJointPID.calculate(CurrentL1Angle, getTargetLowerAngle());
        double L2MotorPower = AlgaeArmSettings.UpperJointPID.calculate(CurrentL2Angle, getTargetUpperAngle());
        
        // Add the gravity/acceleration compensation power

        // Give power to motors
        /* 
        motor.set();
        otherMotor.set();
        */

    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }








    // KINEMATIC AND INVERSE KINEMATIC METHODS
    
    /**
     * Gets the current height and distance forward of the end of the arm based from the lower joint.
     */
    public Vector2d getCurrentPoint() {
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(CurrentL1Angle) + AlgaeArmConstants.UpperSegmentLength * Math.cos(Math.PI - CurrentL2Angle - CurrentL1Angle),
            AlgaeArmConstants.LowerSegmentLength * Math.sin(CurrentL1Angle) + AlgaeArmConstants.UpperSegmentLength * Math.sin(Math.PI + CurrentL2Angle + CurrentL1Angle)
        );
    }
    /**
     * Gets the current height and distance forward of the end of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getCurrentPointFromRobot() {
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(CurrentL1Angle) + AlgaeArmConstants.UpperSegmentLength * Math.cos(Math.PI - CurrentL2Angle - CurrentL1Angle) + AlgaeArmConstants.LowerJointForward,
            AlgaeArmConstants.LowerSegmentLength * Math.sin(CurrentL1Angle) + AlgaeArmConstants.UpperSegmentLength * Math.sin(Math.PI + CurrentL2Angle + CurrentL1Angle) + AlgaeArmConstants.LowerJointHeight
        );
    }

    /**
     * Gets the target height and distance forward of the end of the arm based from the lower joint.
     */
    public Vector2d getTarget() {
        return Target;
    }
    /**
     * Gets the target height and distance forward of the end of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getTargetFromRobot() {
        return Target.plus(new Vector2d(AlgaeArmConstants.LowerJointForward, AlgaeArmConstants.LowerJointHeight));
    }

    /**
     * Gets the current height and distance forward of the second joint of the arm based from the lower joint.
     */
    public Vector2d getCurrentMiddlePoint() {
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(CurrentL1Angle),
            AlgaeArmConstants.LowerSegmentLength * Math.sin(CurrentL1Angle)
        );
    }
    /**
     * Gets the current height and distance forward of the second joint of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getCurrentMiddlePointFromRobot() {
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(CurrentL1Angle) + AlgaeArmConstants.LowerJointForward,
            AlgaeArmConstants.LowerSegmentLength * Math.sin(CurrentL1Angle) + AlgaeArmConstants.LowerJointHeight
        );
    }
    /**
     * Gets the target height and distance forward of the second joint of the arm based from the lower joint.
     */
    public Vector2d getTargetMiddlePoint() {
        double L1Angle = getTargetLowerAngle();
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(L1Angle),
            AlgaeArmConstants.LowerSegmentLength * Math.sin(L1Angle)
        );
    }
    /**
     * Gets the target height and distance forward of the second joint of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getTargetMiddlePointFromRobot() {
        double L1Angle = getTargetLowerAngle();
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(L1Angle) + AlgaeArmConstants.LowerJointForward,
            AlgaeArmConstants.LowerSegmentLength * Math.sin(L1Angle) + AlgaeArmConstants.LowerJointHeight
        );
    }

    public double getCurrentLowerAngle() { return Functions.normalizeAngle(CurrentL1Angle); }
    public double getCurrentLowerAngleFromRobot() { return Functions.normalizeAngle(CurrentL1Angle - Math.PI/2); }

    public double getCurrentUpperAngle() { return Functions.normalizeAngle(CurrentL2Angle); }
    public double getCurrentUpperAngleFromRobot() { return Functions.normalizeAngle((Math.PI - CurrentL2Angle) - getCurrentLowerAngleFromRobot()); }

    public double getTargetLowerAngle() { return Functions.normalizeAngle(Target.getAngle() + Math.asin((AlgaeArmConstants.UpperSegmentLength * Math.sin(getTargetUpperAngle())) / Target.getMag())); }
    public double getTargetLowerAngleFromRobot() { return Functions.normalizeAngle(getTargetLowerAngle() - Math.PI/2); }

    public double getTargetUpperAngle() {
        double result = Math.acos(((Target.x * Target.x + Target.y * Target.y) - AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength * AlgaeArmConstants.UpperSegmentLength) 
            / (-2 * AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.UpperSegmentLength));
        if (RightBias) return Functions.normalizeAngle(-1 * result);
        else return Functions.normalizeAngle(result);
    }
    public double getTargetUpperAngleFromRobot() { return Functions.normalizeAngle((Math.PI - getTargetUpperAngle()) - getTargetLowerAngleFromRobot()); }



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
    public void setTargetLowerAngle(double lowerTargetAngle) { setTargetAnglesFromRobot(lowerTargetAngle - Math.PI/2, getCurrentUpperAngleFromRobot()); }
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
    public void setTargetLowerAngleFromRobot(double lowerTargetAngle) { setTargetAnglesFromRobot(lowerTargetAngle, getCurrentUpperAngleFromRobot()); }
    /**
     * pi is straight up for lower arm, pi is straight out relative to lower arm angle for the upper segment
     */
    public void setTargetUpperAngleFromRobot(double upperTargetAngle) { setTargetAnglesFromRobot(getCurrentLowerAngleFromRobot(), upperTargetAngle); }
    /**
     * An angle of pi is straight up for lower Arm while an angle of pi is straight relative to the lower arm for the upper segment
     */
    public void setTargetAngles(double lowerTargetAngle, double upperTargetAngle) {
        Target = new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(lowerTargetAngle) + AlgaeArmConstants.UpperSegmentLength * Math.cos(Math.PI - upperTargetAngle - lowerTargetAngle),
            AlgaeArmConstants.LowerSegmentLength * Math.sin(lowerTargetAngle) + AlgaeArmConstants.UpperSegmentLength * Math.sin(Math.PI + upperTargetAngle + lowerTargetAngle)
        );
    }
    /**
     * An angle of pi/2 is straight up for both segments
     */
    public void setTargetAnglesFromRobot(double lowerTargetAngle, double upperTargetAngle) {
        double FromRobotLowerAngle = Functions.normalizeAngle(lowerTargetAngle + Math.PI/2);
        double FromRobotUpperAngle = Functions.normalizeAngle((Math.PI + upperTargetAngle) - FromRobotLowerAngle + Math.PI/2);
        Target = new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(FromRobotLowerAngle) + AlgaeArmConstants.UpperSegmentLength * Math.cos(Math.PI - FromRobotUpperAngle - FromRobotLowerAngle),
            AlgaeArmConstants.LowerSegmentLength * Math.sin(FromRobotLowerAngle) + AlgaeArmConstants.UpperSegmentLength * Math.sin(Math.PI + FromRobotUpperAngle + FromRobotLowerAngle)
        );
    }




    // OTHER METHODS

    public boolean isCloseToTarget() {
        return Target.distFrom(getCurrentPoint()) < AlgaeArmSettings.PositionTolerance;
    }

}
