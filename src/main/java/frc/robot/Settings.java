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

    public static boolean useFlightStick = false;

    public static class CoralSystemSettings {
        public static double kSE = 0.0001;
        public static double kGE = 0.00;
        public static double kVE = 0.1;

        public static double kSA = 0.001; // 0.001
        public static double kGA = 0.05; // 0.05
        public static double kVA = 0.05; // 0.05

        public static double elevatorTolerance =4;
        public static double armTolerance =5;
        public static double elevatorSpeedControl =1;

        public static double coralClawStowedAngle = 90;
    }

    public static class CoralClawSettings {
        public static double startRoll = 0;
        public static double startPitch = 0;

        // Temporary single motor diffy control
        public static PIDController LeftDiffyPID = new PIDController(0, 0, 0); // 0, 0, 0
        public static double startRotatePosition = 0;
        public static double secondRotatePosition = 90;

        //Diffy Motor PID
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;

        // feedforward values which one motor needs to be set really high becauswe Cole and Micah suck at building and designing
        public static double RFF = 0.0;
        public static double LFF = 0.0;

        //Limits - Degrees
        public static double maxPitch = 90.0; // degrees
        public static double minPitch = -90.0;
        public static double rollRange = 180.0;

        public static double intakePower = 0.2;
        public static double holdPower = 0.0; // TODO add this
        public static double outtakePower = -0.5;
        //public static int intakeSmartStallCurrent = 4;
        //public static int intakeSecondaryCurrent = 5;

        //public static int diffyMotorSmartStallCurrent = 25;

        public static double testingPitch = 0;
        public static double testingRoll = 0;

    }
    public static class ClimbingSettings{
        public static double climbingSpeed = 0.1; // TODO:undo
    }

    public static class AlgaeArmSettings {

        public static double AlgaeArmLowerJointStartAngle = Math.toRadians(128.5); // 128.5
        public static double AlgaeArmUpperJointStartAngle = Math.toRadians(146.5); // 146.5

        // For motion profile
        public static double maxAcceleration = 0; // in/s^2
        public static double maxDeceleration = 0; // in/s^2
        public static double maxSpeed = 0; // in/s

        public static PIDController LowerJointPID = new PIDController(1, 0, 0); // 3.5, 0, 0
        public static PIDController UpperJointPID = new PIDController(0.8, 0, 0); // 0.6, 0, 0
        public static double voltageTolerance =0.05; //tolerance for PID to stop jitter movements and to 0 out voltage

        // Feedforward
        public static boolean useFeedforward = false; //TODO finish tuning this
        public static ArmFeedforward L1Feedforward = new ArmFeedforward(0, 0, 0);
        public static ArmFeedforward L2Feedforward = new ArmFeedforward(0, 0, 0);

        // Limits
        public static double maxAngleLowerJoint = Math.toRadians(120);
        public static double minAngleLowerJoint = Math.toRadians(10);
        public static double maxAngleUpperJointFromLower = Math.toRadians(170); // 0 being straight
        public static double minAngleUpperJointFromLower = Math.toRadians(-170);
        public static double maxDistanceInX = -8; // prevents hitting the hanging mechanism
        public static double maxDistanceOutX = 17.5; // expansion limit 19.5 is at actual limit
        public static double maxDistanceDownY = -AlgaeArmConstants.LowerJointHeight + 2; // prevents hitting the floor

        public static double PositionTolerance = 5; // in inches for checking when arm has reached target position

        public static double maxJoystickMovementSpeed = 12; // in/s
        public static double temporaryStaticPower = 0;

    }

    public static class AlgaeRollerSettings {
        public static double IntakePower = 0.3;
        public static double HoldPower = 0.15;
        public static double OuttakePower = -0.5;
        //public static int maxSmartCurrent = 15;
        //public static int secondaryCurrentLimit = 20;
    }
}