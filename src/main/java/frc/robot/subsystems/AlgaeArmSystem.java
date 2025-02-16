package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import frc.robot.Settings.AlgaeArmSettings;
import frc.robot.utility.MotionProfile2d;
import frc.robot.utility.Vector2d;


public class AlgaeArmSystem extends SubsystemBase {

    public double TargetLowerJointAngle, TargetUpperJointAngle; // radians

    private double CurrentLowerJointAngle, CurrentUpperJointAngle; // radians
    private Vector2d Target; // inches

    private DoubleSupplier LowerJointEncoder, UpperJointEncoder;

    private MotionProfile2d motionProfile;



    public AlgaeArmSystem() {
        // initialize motors

        // initialize PIDs

        LowerJointEncoder = () -> 0; //TODO: add get Encoder position and conversion to radians
        UpperJointEncoder = () -> 0; //TODO: add get Encoder position and conversion to radians

        CurrentLowerJointAngle = LowerJointEncoder.getAsDouble();
        CurrentUpperJointAngle = LowerJointEncoder.getAsDouble();

        // Get start position
        Target = new Vector2d(); // TODO: set this to something before running

        motionProfile = new MotionProfile2d(Target, AlgaeArmSettings.maxAcceleration, AlgaeArmSettings.maxDeceleration, AlgaeArmSettings.maxSpeed);
    }


    @Override
    public void periodic() { // Main run method

        //TODO: We only need this if we can and want to tune these variables while driving
        motionProfile.updateSettings(AlgaeArmSettings.maxAcceleration, AlgaeArmSettings.maxDeceleration, AlgaeArmSettings.maxSpeed);

        // Update encoders
        CurrentLowerJointAngle = LowerJointEncoder.getAsDouble();
        CurrentUpperJointAngle = LowerJointEncoder.getAsDouble();


        // Get moving target from motion profile

        // Make sure target is within reachable positions and won't clip into things

        // Tell inverse kinematics statement to get new target joint angles

        // Pass target joint angles to PID and add the gravity/acceleration compensation power

        // Give power to motors

    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }


    // inverse kinematics methods




}
