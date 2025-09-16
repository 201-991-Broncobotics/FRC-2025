package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings.AlgaeArmSettings;
import frc.robot.subsystems.CommandSwerveDrivetrain.gyroData;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.ElapsedTime.Resolution;
import frc.robot.utility.Functions;

public class AlgaeArm extends SubsystemBase {
    
    private TalonFX pivotMotor;
    private SparkFlex algaeRoller;
    private SparkFlexConfig algaeRollerConfig;

    private DoubleSupplier algaePivotEncoder;

    private DoubleSupplier ManualControlAxis = () -> 0;
    private boolean GoToPosition = false; // whether or not the manual control controls the power up and down or sets the position

    private double PivotMotorPower, AlgaeRollerPower;

    private double TargetPivotAngle;

    private final double pivotGearRatio = 1.0/5.0 * 1.0/5.0;

    private double pivotZeroAngle = 0;

    private ElapsedTime runTime;
    private double frameTime = 0;

    private boolean enabled = true;
    
    private double setPresetPosition = AlgaeArmSettings.PresetPickupAngle;
    private double presetTolerance = Math.toRadians(10); // radians


    public AlgaeArm() {
        runTime = new ElapsedTime(Resolution.SECONDS);
        
        algaeRollerConfig = new SparkFlexConfig();

        TargetPivotAngle = AlgaeArmSettings.AlgaePivotStartAngle;


        pivotMotor = new TalonFX(MotorConstants.algaePivotID);
        pivotMotor.setNeutralMode(NeutralModeValue.Coast);
        algaeRoller = new SparkFlex(MotorConstants.algaeRollerID, MotorType.kBrushless);
        algaeRollerConfig.idleMode(IdleMode.kCoast);
        algaeRollerConfig.smartCurrentLimit(AlgaeArmSettings.algaeRollerStallCurrent);
        algaeRoller.configure(algaeRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        PivotMotorPower = 0;
        AlgaeRollerPower = 0;

        pivotZeroAngle = pivotMotor.getPosition().getValueAsDouble();

        algaePivotEncoder = () -> (pivotMotor.getPosition().getValueAsDouble() - pivotZeroAngle) * pivotGearRatio * 2 * Math.PI + AlgaeArmSettings.AlgaePivotStartAngle;


        SmartDashboard.putNumber("Tune Algae Pivot kP", AlgaeArmSettings.AlgaePivotPID.getP());
        SmartDashboard.putNumber("Tune Algae Pivot kI", AlgaeArmSettings.AlgaePivotPID.getI());
        SmartDashboard.putNumber("Tune Algae Pivot kD", AlgaeArmSettings.AlgaePivotPID.getD());

    }


    public void update() {

        if (!GoToPosition) {
            TargetPivotAngle += ManualControlAxis.getAsDouble() * AlgaeArmSettings.manualControlSpeed * frameTime;
        } else {
            TargetPivotAngle = ManualControlAxis.getAsDouble() * (AlgaeArmSettings.MaxPivotAngle - AlgaeArmSettings.MinPivotAngle) + AlgaeArmSettings.MinPivotAngle;

            if (Math.abs(setPresetPosition - TargetPivotAngle) < presetTolerance) TargetPivotAngle = setPresetPosition;
        }
        

        TargetPivotAngle = Functions.minMaxValue(AlgaeArmSettings.MinPivotAngle, AlgaeArmSettings.MaxPivotAngle, TargetPivotAngle);

        double currentPivotAngle = algaePivotEncoder.getAsDouble();
        PivotMotorPower = AlgaeArmSettings.AlgaePivotPID.calculate(algaePivotEncoder.getAsDouble(), TargetPivotAngle) + AlgaeArmSettings.gravityPower * Math.sin(currentPivotAngle) + (gyroData.accelX * AlgaeArmSettings.gravityPower * Math.cos(currentPivotAngle) * (AlgaeArmSettings.useDriveCompensation? 1.0:0.0));

        //if (algaeRoller.getOutputCurrent() > AlgaeArmSettings.algaeRollerHasIntakedCurrent && AlgaeRollerPower == AlgaeArmSettings.IntakePower) {
        //    AlgaeRollerPower = AlgaeArmSettings.HoldPower;
        //}

        if (enabled) {
            pivotMotor.set(PivotMotorPower);
        } else {
            pivotMotor.set(0);
        }

        algaeRoller.set(-AlgaeRollerPower);

    }

    @Override
    public void periodic() { 
        frameTime = runTime.time();
        runTime.reset();

        SmartDashboard.putNumber("Algae Current Pivot Angle", Math.toDegrees(algaePivotEncoder.getAsDouble()));
        SmartDashboard.putNumber("Algae Target Pivot Angle", Math.toDegrees(TargetPivotAngle));
        SmartDashboard.putNumber("Algae Pivot Motor Current", pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Algae Pivot Motor Power", PivotMotorPower);

        SmartDashboard.putNumber("Algae Roller Power", AlgaeRollerPower);
        SmartDashboard.putNumber("Algae Roller Motor Current", algaeRoller.getOutputCurrent());


        AlgaeArmSettings.AlgaePivotPID.setP(SmartDashboard.getNumber("Tune Algae Pivot kP", AlgaeArmSettings.AlgaePivotPID.getP()));
        AlgaeArmSettings.AlgaePivotPID.setI(SmartDashboard.getNumber("Tune Algae Pivot kI", AlgaeArmSettings.AlgaePivotPID.getI()));
        AlgaeArmSettings.AlgaePivotPID.setD(SmartDashboard.getNumber("Tune Algae Pivot kD", AlgaeArmSettings.AlgaePivotPID.getD()));

    }


    public void Enable() { enabled = true; }
    public void Disable() { enabled = false; }
    public void toggleEnabled() { enabled = !enabled; }


    public void presetIntakePosition() {
        TargetPivotAngle = AlgaeArmSettings.PresetPickupAngle;
        AlgaeRollerPower = AlgaeArmSettings.IntakePower;

        setPresetPosition = AlgaeArmSettings.PresetPickupAngle;
    }

    public void presetStorePosition() {
        TargetPivotAngle = AlgaeArmSettings.PresetStoredAngle;
        AlgaeRollerPower = AlgaeArmSettings.HoldPower;

        setPresetPosition = AlgaeArmSettings.PresetStoredAngle;
    }

    public void presetOuttakePosition() {
        TargetPivotAngle = AlgaeArmSettings.PresetOuttakeAngle;

        setPresetPosition = AlgaeArmSettings.PresetOuttakeAngle;
        // AlgaeRollerPower = AlgaeArmSettings.OuttakePower;
    }

    public void setManualControl(DoubleSupplier controlAxis, boolean goToPosition) {
        ManualControlAxis = controlAxis;
        GoToPosition = goToPosition;
    }

    public void setManualControl(DoubleSupplier controlAxis) {
        ManualControlAxis = controlAxis;
    }

    public void stopRoller() { AlgaeRollerPower = 0; }
    public void intakeRoller() { AlgaeRollerPower = AlgaeArmSettings.IntakePower; }
    public void holdRoller() { AlgaeRollerPower = AlgaeArmSettings.HoldPower; }
    public void outtakeRoller() { AlgaeRollerPower = AlgaeArmSettings.OuttakePower; }


}
