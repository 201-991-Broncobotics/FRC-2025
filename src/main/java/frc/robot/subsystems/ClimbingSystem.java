package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings.ClimbingSettings;

public class ClimbingSystem extends SubsystemBase {

    private double climbingSpeed;
    private TalonFX climbingMotor;

    private double ClimbingPower = 0;


    public ClimbingSystem() {
        climbingMotor = new TalonFX(MotorConstants.climbingMotorID);
        climbingSpeed = ClimbingSettings.climbingSpeed;
        ClimbingPower = 0;
    }

    public void update() {
        climbingMotor.set(ClimbingPower);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Motor Current", climbingMotor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void StartClimbing() {
        ClimbingPower = climbingSpeed;
    }

    public void StartUnclimbing() {
        ClimbingPower = -climbingSpeed;
    }

    public void StopClimbing() {
        ClimbingPower = 0;
    }

}
