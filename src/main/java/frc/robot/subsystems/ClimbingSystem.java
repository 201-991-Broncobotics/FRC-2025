package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings.ClimbingSettings;

public class ClimbingSystem extends SubsystemBase {

    private double climbingSpeed;
    //private SparkMax climbingMotor;
    private StatusCode climbingMotorConfig;
    private TalonFX climbingMotor;
    private CurrentLimitsConfigs currentLimitConfigs;

    private double ClimbingPower = 0;

    //private SparkMaxConfig climbMotorConfig;


    public ClimbingSystem() {
        //climbingMotor = new SparkMax(MotorConstants.climbingMotorID, MotorType.kBrushless);
        //climbMotorConfig = new SparkMaxConfig();

        //climbMotorConfig.idleMode(IdleMode.kBrake);

        //climbingMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climbingMotor = new TalonFX(MotorConstants.climbingMotorID);
        //climbingMotor.setControl(climbingMotorConfig);

        currentLimitConfigs = new CurrentLimitsConfigs() // might not work
            .withStatorCurrentLimit(Amps.of(20))
            .withStatorCurrentLimitEnable(true);

        climbingMotorConfig = climbingMotor.getConfigurator().apply(currentLimitConfigs);

        climbingMotor.setNeutralMode(NeutralModeValue.Coast);
        

        climbingSpeed = ClimbingSettings.climbingSpeed;
        ClimbingPower = 0;
    }

    public void update() {
        if (climbingMotor.getPosition().getValueAsDouble() > ClimbingSettings.maxEncoderPosition && ClimbingPower > 0) ClimbingPower = 0;
        if (climbingMotor.getPosition().getValueAsDouble() < ClimbingSettings.minEncoderPosition && ClimbingPower < 0) ClimbingPower = 0;
        climbingMotor.set(ClimbingPower);
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Motor Current", climbingMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Climb Encoder", climbingMotor.getPosition().getValueAsDouble());
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
