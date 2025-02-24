package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.robot.Constants.MotorConstants;

public class CoralArmSystem extends SubsystemBase {

    private double TargetElevatorHeight, TargetArmAngle, ElevatorError;
    private double CurrentElevatorHeight, CurrentArmAngle;

    private DoubleSupplier elevatorControl;

    private ElevatorFeedforward feedforward;

    private TalonFX leftElevator;
    private TalonFX rightElevator;
    private TalonFX coralPivot;


    public CoralArmSystem() {
        leftElevator = new TalonFX(MotorConstants.coralLeftElevatorID);
        rightElevator = new TalonFX(MotorConstants.coralRightElevatorID);
        coralPivot = new TalonFX(MotorConstants.coralPivotID);
        feedforward = new ElevatorFeedforward(Settings.CoralSystemSettings.kS, Settings.CoralSystemSettings.kG, Settings.CoralSystemSettings.kV, 0);

    }
    public void setPos(double pos){
        TargetElevatorHeight = pos;
    }


    @Override
    public void periodic() {
        
        //Update position
        if (leftElevator.getPosition().getValueAsDouble()<0) 
            CurrentElevatorHeight =0;
        else CurrentElevatorHeight = leftElevator.getPosition().getValueAsDouble(); //add offset later

        //Calculate voltage
        ElevatorError=TargetElevatorHeight-CurrentElevatorHeight;
        //Move motors
        if(Math.abs(ElevatorError)<Settings.CoralSystemSettings.elevatorTolerance) {
            leftElevator.set(-feedforward.calculate(0));
        rightElevator.set(feedforward.calculate(0));
        }
        else{
            leftElevator.set(-feedforward.calculate(ElevatorError/Settings.CoralSystemSettings.elevatorSpeedControl));
            rightElevator.set(feedforward.calculate(ElevatorError/Settings.CoralSystemSettings.elevatorSpeedControl));
        }
        
        //Smart Dashboard updates
        SmartDashboard.putNumber("Right Elevator", CurrentElevatorHeight);
        SmartDashboard.putNumber("Right ElevatorAct ", rightElevator.getPosition().getValueAsDouble());
    }
    public boolean atPosition(){
        if(Math.abs(ElevatorError)<Settings.CoralSystemSettings.elevatorTolerance)
        return true;
        return false;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
