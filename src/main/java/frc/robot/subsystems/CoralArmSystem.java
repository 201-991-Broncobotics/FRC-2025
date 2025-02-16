package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralArmSystem extends SubsystemBase {

    private double TargetElevatorHeight, TargetArmAngle;

    private double CurrentElevatorHeight, CurrentArmAngle;


    public CoralArmSystem() {

    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
