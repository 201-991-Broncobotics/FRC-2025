package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class CoralArmSystem extends SubsystemBase {

    private double TargetElevatorHeight, TargetArmAngle;

    private double CurrentElevatorHeight, CurrentArmAngle;

    private DoubleSupplier elevatorControl;

    private ElevatorFeedforward feedforward;

    private TalonFX leftElevator;
    private TalonFX rightElevator;
    private TalonFX coralPivot;


    public CoralArmSystem(DoubleSupplier elevatorControl) {
        leftElevator = new TalonFX(MotorConstants.coralLeftElevatorID);
        rightElevator = new TalonFX(MotorConstants.coralRightElevatorID);
        coralPivot = new TalonFX(MotorConstants.coralPivotID);
        this.elevatorControl=elevatorControl;

    }
    public void setPos(double pos){
        TargetElevatorHeight = pos;

    }


    @Override
    public void periodic() {
        leftElevator.set(-elevatorControl.getAsDouble());
        rightElevator.set(elevatorControl.getAsDouble());
        //Update position
        CurrentElevatorHeight = rightElevator.getPosition().getValueAsDouble(); //add offset later

        //Smart Dashboard updates
        
        SmartDashboard.putNumber("Left Elevator", leftElevator.getPosition().getValueAsDouble());
        SmartDashboard.putString("Right Elevator", ""+CurrentElevatorHeight);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
