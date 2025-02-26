package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings.CoralClawSettings;

public class CoralClaw extends SubsystemBase {

    private double CurrentRightAngle, CurrentLeftAngle;
    private double TargetRightAngle, TargetLeftAngle;


    private DoubleSupplier RightEncoder, LeftEncoder;
    
    private double Rotation = 0, Pitch = 0;

    private Spark leftMotor, rightMotor;


    public CoralClaw(double startRotation, double startPitch) {
        Rotation = startRotation;
        Pitch = startPitch;

        rightMotor = new Spark(Constants.MotorConstants.rightDiffyID);
        leftMotor = new Spark(Constants.MotorConstants.leftDiffyID);

        RightEncoder = () -> 0; // TODO: Setup encoders
        LeftEncoder = () -> 0; 
        
    }


    @Override
    public void periodic() {
        CurrentRightAngle = RightEncoder.getAsDouble();
        CurrentLeftAngle = LeftEncoder.getAsDouble();
        
        TargetRightAngle = Pitch + Rotation;
        TargetLeftAngle = Pitch - Rotation;

        rightMotor.set(0);


    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }




    public void setLimits(double maxPitch, double minPitch, double wristAngleRange) {
        CoralClawSettings.maxPitch = maxPitch;
        CoralClawSettings.minPitch = minPitch;
        CoralClawSettings.wristAngleRange = wristAngleRange;
    }

    public void setTo(double rotation, double pitch) {
        this.Rotation = rotation;
        this.Pitch = pitch;
    }

}
