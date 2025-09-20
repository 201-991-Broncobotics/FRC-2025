// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class MotorConstants{

        public static final int algaePivotID = 17;

        public static final int coralLeftElevatorID = 9;
        public static final int coralRightElevatorID = 11;
        public static final int coralPivotID =18;

        // NEOs
        public static final int algaeRollerID = 19;
    }
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class AlgaeArmConstants {
        public static final double LowerSegmentLength = 20.375; // inches
        public static final double UpperSegmentLength = 23; // inches

        public static final double LowerJointForward = 9.25; // inches between lower joint and center of robot frame
        public static final double LowerJointHeight = 14.227224; // inches between lower joint and floor

        public static final double maxVoltage = 12; 
        public static final double minVoltage = 6; 


        // Gravity compensation related
        public static final double L1Mass = 3.4505187; // lbs
        public static final double L2Mass = 6.0573773; // lbs
        public static final double L1CoM = 11.672603; // inches along segment length 
        public static final double L2CoM = 15.3324520; // inches along segment length 


    }

    public static class CoralSystemConstants {
        public static final double CoralArmGearRatio = (1.0/108.0 * 4/3);
    }

}
