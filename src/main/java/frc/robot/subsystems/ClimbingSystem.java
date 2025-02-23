package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings.ClimbingSettings;

public class ClimbingSystem extends SubsystemBase {

    private double climbingSpeed;
    private TalonFX climbingMotor;
    private BooleanSupplier controlUp;
    private BooleanSupplier controlDown;


    public ClimbingSystem(BooleanSupplier controlUp, BooleanSupplier controlDown) {
        climbingMotor = new TalonFX(MotorConstants.climbingMotorID);
        climbingSpeed = ClimbingSettings.climbingSpeed;
        this.controlUp = controlUp;
        this.controlDown = controlDown;
    }


    @Override
    public void periodic() {
        if(controlUp.getAsBoolean())
        climbingMotor.set(climbingSpeed);
        else if(controlDown.getAsBoolean())
        climbingMotor.set(-climbingSpeed);
        else
        climbingMotor.set(0);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
