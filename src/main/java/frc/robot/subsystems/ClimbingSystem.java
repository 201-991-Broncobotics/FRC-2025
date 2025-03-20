package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings.ClimbingSettings;

public class ClimbingSystem extends SubsystemBase {

    private double climbingSpeed;
    private SparkFlex climbingMotor;

    private double ClimbingPower = 0;

    private SparkFlexConfig climbMotorConfig;


    public ClimbingSystem() {
        climbingMotor = new SparkFlex(36, MotorType.kBrushless);
        climbMotorConfig = new SparkFlexConfig();

        climbMotorConfig.idleMode(IdleMode.kBrake);
        climbingMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        climbingSpeed = ClimbingSettings.climbingSpeed;
        ClimbingPower = 0;
    }

    public void update() {
        climbingMotor.set(ClimbingPower);
    }


    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Climb Motor Current", climbingMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Climb Encoder", -climbingMotor.getAbsoluteEncoder().getPosition());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void StartClimbing() {
        ClimbingPower = -climbingSpeed;
    }

    public void StartUnclimbing() {
        ClimbingPower = climbingSpeed;
    }

    public void StopClimbing() {
        ClimbingPower = 0;
    }

}
