package frc.robot;

import java.util.Vector;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.utility.CoralSystemPreset;
import frc.robot.utility.PIDControllerSettingsReference;
import frc.robot.utility.Vector2d;


/**
 * The point of this class is to house all the variables that may need to be tuned or changed
 * often so that they are easier to find than scrolling through a massive constants class
 */
public class Settings {

    public static boolean tuningTelemetryEnabled = true;

    public static class CoralSystemSettings {

        // public static double elevatorTolerance =.8;
        public static double armTolerance =5;
        // public static double elevatorSpeedControl =1;
        public static double elevatorRotationsToInches =(1.0/20.0)/*gear ratio*/ *(1.757*Math.PI/*circumference of the sprocket's pitch*/)*2;

        public static double startingPosition = 0;
        public static double maxHeight = 48;
        public static double minHeight = 0;

        public static PIDControllerSettingsReference ElevatorReferencePID = new PIDControllerSettingsReference(
                0.03,
                0,
                0,
                0,
                minHeight,
                maxHeight,
                0,
                0.2,
                0,
                0,
                0,
                .8,
                0,
                0,
                true,
                false);

        

        public static double manualControlSpeed = 45; // max speed in inches per second 


        public static double delayBeforeStaging = 750; // milliseconds that after holding the change stage button, will cause it to skip to max/min stage


        public static double lowerLiftRange = Math.toRadians(35); // range where the pivot is stopped and the elevator is moved up to prevent collisions
        public static double upperLiftRange = Math.toRadians(-85);
        public static double liftRangeHeight = 25;

    }

    public static class CoralClawSettings {

        public static int CoralPivotCurrentLimit = 20;
        public static PIDController CoralPivotPID = new PIDController(0.0, 0, 0); // 0.7, 0, 0
        public static double maxAngle = 90;
        public static double minAngle = -93;
        public static double manualPivotSpeed = Math.toRadians(70); // radians per second


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
        public static double holdPower = 0.05; 
        public static double outtakePower = -0.5;
        //public static int intakeSmartStallCurrent = 4;
        //public static int intakeSecondaryCurrent = 5;

        //public static int diffyMotorSmartStallCurrent = 25;

        public static double testingPitch = 0;
        public static double testingRoll = 0;

    }
    public static class ClimbingSettings{
        public static double climbingSpeed = 1;

        public static double maxEncoderPosition = 85;
        public static double minEncoderPosition = 0;
    }

    public static class AlgaeArmSettings {

        public static double AlgaePivotStartAngle = Math.toRadians(95); // 150 before elevator support

        // Presets
        public static double PresetPickupAngle = Math.toRadians(25);
        public static double PresetStoredAngle = Math.toRadians(90);
        public static double PresetOuttakeAngle = Math.toRadians(80);

        // Limits
        public static double MaxPivotAngle = Math.toRadians(95);
        public static double MinPivotAngle = Math.toRadians(0);

        public static PIDController AlgaePivotPID = new PIDController(0.35, 0, 0); // 0, 0, 0
        public static PIDController AlgaeRollerPID = new PIDController(0.0, 0, 0); // 0, 0, 0
        
        public static double IntakePower = 0.4;
        public static double HoldPower = 0.2;
        public static double OuttakePower = -0.6;

        public static int algaeRollerHasIntakedCurrent = 40;
        public static int algaeRollerStallCurrent = 22;

        public static double manualControlSpeed = Math.toRadians(105); // max speed in radians per second 
        
        // gravity power
        public static double gravityPower = 0.0;
        public static boolean useDriveCompensation = false; // helps compensate for accelerating

    }

    public static class AutoTargetingSettings {

        public static boolean AutoAimingEnabled = true;
        public static PIDController AutoTurningPID = new PIDController(0, 0, 0);
        public static PIDController AutoDrivingPID = new PIDController(0, 0, 0);
        public static double AutoDrivingMaxPower = 0.5;

        public static boolean AutoDrivingEnabled = true;
        public static double AutoDrivingPower = 0;
        public static double targetPercentageOfVisionBlocked = 0.2;

        public static double searchingSpeed = 0.5;

        public static double leftCorrectX = 0;
        public static double rightCorrectX = 0;

    }

    public static class CoralSystemPresets {

        // Elevator Height (inches - luckily not in Christian units), Pivot Pitch (degrees)
        public static CoralSystemPreset FunnelIntake = new CoralSystemPreset(20, -90);
        //public static CoralSystemPreset CoralStationIntake = new CoralSystemPreset(11, 75);
        public static CoralSystemPreset L4Reef = new CoralSystemPreset(40, 70);
        public static CoralSystemPreset L3Reef = new CoralSystemPreset(35, 70);
        public static CoralSystemPreset L2Reef = new CoralSystemPreset(25, 70);
        // public static CoralSystemPreset L1Reef = new CoralSystemPreset(0, 80);
        public static CoralSystemPreset Stowed = new CoralSystemPreset(0, 90);

    }


    public static class AutoDrivingSettings {

        // in meters - 0.831659625 m is distance from center to outside of bumper
        public static double OutwardFromCenter = 0.831659625 + ((17.819104 + 0)/39.37); // distance outward from center of the reef for each spot
        public static double LeftFromCenter = ((13 + 0)/39.37)/2.0; // distance left from center of the reef for each spot
        public static double RightFromCenter = ((13 + 0)/39.37)/2.0; // distance right from center of the reef for each spot
    }
}