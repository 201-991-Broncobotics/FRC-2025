package frc.robot.subsystems.subsubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralClaw extends SubsystemBase {

    private double TargetRightAngle, TargetLeftAngle;

    private double CurrentRightAngle, CurrentLeftAngle;


    public CoralClaw() {

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
