package frc.robot;

import edu.wpi.first.math.controller.PIDController;

/**
 * The point of this class is to house all the variables that may need to be tuned or changed
 * often so that they are easier to find than scrolling through a massive constants class
 */
public class Settings {

    public static class AlgaeArmSettings {

        // For motion profile
        public static double maxAcceleration = 0; // in/s^2
        public static double maxDeceleration = 0; // in/s^2
        public static double maxSpeed = 0; // in/s

        public static PIDController LowerJointPID = new PIDController(0, 0, 0);
        public static PIDController UpperJointPID = new PIDController(0, 0, 0);

        public static double PositionTolerance = 1; // in inches for checking when arm has reached target position

    }
}