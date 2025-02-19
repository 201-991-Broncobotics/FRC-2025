package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.AlgaeArmConstants;

/**
 * The point of this class is to house all the variables that may need to be tuned or changed
 * often so that they are easier to find than scrolling through a massive constants class
 */
public class Settings {

    public static class AlgaeArmSettings {

        public static double AlgaeArmLowerJointStartAngle = Math.toRadians(180);
        public static double AlgaeArmUpperJointStartAngle = Math.toRadians(5);

        // For motion profile
        public static double maxAcceleration = 0; // in/s^2
        public static double maxDeceleration = 0; // in/s^2
        public static double maxSpeed = 0; // in/s

        public static PIDController LowerJointPID = new PIDController(0, 0, 0);
        public static PIDController UpperJointPID = new PIDController(0, 0, 0);
        public static double voltageTolerance =0.05; //tolerance for PID to stop jitter movements and to 0 out voltage

        // Limits
        public static double maxAngleLowerJoint = Math.toRadians(330);
        public static double minAngleLowerJoint = Math.toRadians(5);
        public static double maxAngleUpperJoint = Math.toRadians(190);
        public static double minAngleUpperJoint = Math.toRadians(50);
        public static double maxDistanceInX = -8; // prevents hitting the hanging mechanism
        public static double maxDistanceOutX = 19.5; // expansion limit
        public static double maxDistanceDownY = -AlgaeArmConstants.LowerJointHeight + 2; // prevents hitting the floor

        public static double PositionTolerance = 1; // in inches for checking when arm has reached target position

        public static double maxJoystickMovementSpeed = 5; // in/s

    }
}