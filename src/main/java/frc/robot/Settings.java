package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.AlgaeArmConstants;

/**
 * The point of this class is to house all the variables that may need to be tuned or changed
 * often so that they are easier to find than scrolling through a massive constants class
 */
public class Settings {

    public static boolean tuningTelemetryEnabled = true;

    public static class CoralSystemSettings {
        public static double kSE = 0.25;
        public static double kGE = 0.01;
        public static double kVE = 0.0;

        public static double kSA = 0.0;
        public static double kGA = 0.0;
        public static double kVA = 0.0;

        public static double elevatorTolerance =2;
        public static double armTolerance =5;
        public static double elevatorSpeedControl =11;
    }

    public static class CoralClawSettings {
        //Right Motor PID & Speed
        public static double RkP = 0.0;
        public static double RkI = 0.0;
        public static double RkD = 0.0;
        public static double Rspeed = 1.0;

        // feedforward values which one motor needs to be set really high becauswe Cole and Micah suck at designing and building
        public static double RFF = 0.0;
        public static double LFF = 0.0;

        //Limits - Degrees
        public static double maxPitch = 90.0;
        public static double minPitch = -90.0;
        public static double rollRange = 90.0;
    }
    public static class ClimbingSettings{
        public static double climbingSpeed = 1;
    }

    public static class AlgaeArmSettings {

        public static double AlgaeArmLowerJointStartAngle = Math.toRadians(90); // 94.4 
        public static double AlgaeArmUpperJointStartAngle = Math.toRadians(90); // 345.5 - 94.4

        // For motion profile
        public static double maxAcceleration = 0; // in/s^2
        public static double maxDeceleration = 0; // in/s^2
        public static double maxSpeed = 0; // in/s

        public static PIDController LowerJointPID = new PIDController(0, 0, 0); // 3.5, 0, 0
        public static PIDController UpperJointPID = new PIDController(0, 0, 0); // 0.6, 0, 0
        public static double voltageTolerance =0.05; //tolerance for PID to stop jitter movements and to 0 out voltage

        // Feedforward
        public static boolean useFeedforward = false; //TODO finish tuning this
        public static ArmFeedforward L1Feedforward = new ArmFeedforward(0, 0, 0);
        public static ArmFeedforward L2Feedforward = new ArmFeedforward(0, 0, 0);

        // Limits
        public static double maxAngleLowerJoint = Math.toRadians(120);
        public static double minAngleLowerJoint = Math.toRadians(-5);
        public static double maxAngleUpperJointFromLower = Math.toRadians(170); // 0 being straight
        public static double minAngleUpperJointFromLower = Math.toRadians(-170);
        public static double maxDistanceInX = -8; // prevents hitting the hanging mechanism
        public static double maxDistanceOutX = 19.5; // expansion limit
        public static double maxDistanceDownY = -AlgaeArmConstants.LowerJointHeight + 2; // prevents hitting the floor

        public static double PositionTolerance = 1; // in inches for checking when arm has reached target position

        public static double maxJoystickMovementSpeed = 5; // in/s
        public static double temporaryStaticPower = 0;

    }

    public static class AlgaeRollerSettings {
        public static double IntakePower = 0.5;
        public static double HoldPower = 0.25;
        public static double OuttakePower = -0.5;
    }
}