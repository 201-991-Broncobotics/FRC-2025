package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class CoralArmSystem extends SubsystemBase {

    private double TargetElevatorHeight, TargetArmAngle;

    private double CurrentElevatorHeight, CurrentArmAngle;
    private TalonFX leftElevator;
    private TalonFX rightElevator;
    private TalonFX coralPivot;


    public CoralArmSystem() {
        leftElevator = new TalonFX(MotorConstants.coralLeftElevatorID);
        rightElevator = new TalonFX(MotorConstants.coralRightElevatorID);
        coralPivot = new TalonFX(MotorConstants.coralPivotID);
    }
    public void setPos(double pos){
        TargetElevatorHeight = pos;
        
    }


    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
