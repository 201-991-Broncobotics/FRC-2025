package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings.AlgaeArmSettings;
import frc.robot.Settings.CoralClawSettings;
import frc.robot.Settings.CoralSystemPresets;
import frc.robot.Settings.CoralSystemSettings;
import frc.robot.utility.CoralSystemPreset;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.Functions;
import frc.robot.utility.ElapsedTime.Resolution;
import frc.robot.utility.AdvancedPIDController;
import frc.robot.utility.PIDControllerSettingsReference;
import frc.robot.Constants.CoralSystemConstants;
import frc.robot.Constants.MotorConstants;

public class CoralElevatorSystem extends SubsystemBase {
    public static int ElevatorStage;

    private double TargetElevatorHeight, ElevatorError;
    private DoubleSupplier CurrentElevatorHeight;

    private ElevatorFeedforward elevatorFeedForward;

    private TalonFX elevatorR, elevatorL, elevatorPivot;
    private TalonFXConfiguration elevatorPivotConfig;
    private CurrentLimitsConfigs currentLimits;
    //private SparkMax elevatorPivot;
    //private SparkMaxConfig elevatorPivotConfig;

    private DoubleSupplier CurrentArmAngle;
    private double TargetArmAngle = Math.toRadians(90);
    private double PivotArmOffset = 0;

    private ElapsedTime runTime;
    private double frameTime = 0;

    private DoubleSupplier ManualControlAxis = () -> 0;
    private boolean GoToPosition = false; // whether or not the manual control controls the power up and down or sets the position

    private int numberOfStages = 5;

    private boolean overrideManualControl = false;
    private double lastManualControl = 0;
    private double overrideOverrideTolerance = 0.05; // units are joystick input

    private ElapsedTime stageChangeButtonTimer;

    private boolean pendingStageChange = false;

    private boolean lastWasStagingUp = false;

    private boolean enabled = true;
    private boolean pivotEnabled = true;

    private DoubleSupplier ManualPivotControl;

    private AdvancedPIDController ElevatorPID;

    private double ElevatorPower = 0;

    private double PivotPower = 0;


    //temp
    //private DoubleSupplier testEle;


    public CoralElevatorSystem() {
        elevatorR = new TalonFX(MotorConstants.coralRightElevatorID);
        elevatorL = new TalonFX(MotorConstants.coralLeftElevatorID);
        elevatorR.setNeutralMode(NeutralModeValue.Brake);
        elevatorL.setNeutralMode(NeutralModeValue.Brake);

        elevatorPivot = new TalonFX(MotorConstants.coralPivotID);//new SparkMax(MotorConstants.coralPivotID, MotorType.kBrushless);

        elevatorPivotConfig = new TalonFXConfiguration();
        currentLimits = new CurrentLimitsConfigs();

        // Set supply current limit
        currentLimits.SupplyCurrentLimitEnable = true;
        currentLimits.SupplyCurrentLimit = 40;

        elevatorPivotConfig.CurrentLimits = currentLimits;

        elevatorPivot.getConfigurator().apply(elevatorPivotConfig); //new SparkMaxConfig();
        /*elevatorPivotConfig.idleMode(IdleMode.kBrake);
        elevatorPivotConfig.smartCurrentLimit(CoralClawSettings.CoralPivotCurrentLimit);
        elevatorPivot.configure(elevatorPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        */
        CurrentArmAngle = () -> 2 * Math.PI * CoralSystemConstants.CoralArmGearRatio * elevatorPivot.getPosition().getValueAsDouble() - PivotArmOffset;
        CurrentElevatorHeight = () -> elevatorR.getPosition().getValueAsDouble()*CoralSystemSettings.elevatorRotationsToInches;
        PivotArmOffset = CurrentArmAngle.getAsDouble() - Math.toRadians(90);


        TargetElevatorHeight = 0.0;
        // elevatorFeedForward = new ElevatorFeedforward(CoralSystemSettings.kSE, CoralSystemSettings.kGE, CoralSystemSettings.kVE);
        ElevatorPID = new AdvancedPIDController(0, 0, 0, CurrentElevatorHeight);
        ElevatorPID.setSettingsTheSameAs(CoralSystemSettings.ElevatorReferencePID);

        runTime = new ElapsedTime(Resolution.SECONDS);

        //put numbers so we can grab latter brr
        SmartDashboard.putNumber("ElevatorPID kP", CoralSystemSettings.ElevatorReferencePID.kP);
        SmartDashboard.putNumber("ElevatorPID kD", CoralSystemSettings.ElevatorReferencePID.kD);
        SmartDashboard.putNumber("ElevatorPID max pos", CoralSystemSettings.ElevatorReferencePID.maxPosition);
        SmartDashboard.putNumber("ElevatorPID max power", CoralSystemSettings.ElevatorReferencePID.maxPower);
        SmartDashboard.putNumber("ElevatorPID max speed", CoralSystemSettings.ElevatorReferencePID.maxSpeed);
        SmartDashboard.putNumber("ElevatorPID max accel", CoralSystemSettings.ElevatorReferencePID.maxAcceleration);
        SmartDashboard.putNumber("ElevatorPID max decel", CoralSystemSettings.ElevatorReferencePID.maxDeceleration);
        SmartDashboard.putNumber("Target Pivot Angle", Math.toDegrees(TargetArmAngle));
        SmartDashboard.putNumber("Current Pivot Angle", Math.toDegrees(CurrentArmAngle.getAsDouble()));
        SmartDashboard.putNumber("Coral Pivot Power", PivotPower);
        SmartDashboard.putNumber("Coral Pivot Angle Actual", elevatorPivot.getPosition().getValueAsDouble()); 

        SmartDashboard.putNumber("Tune Coral Pivot kP", CoralClawSettings.CoralPivotPID.getP());
        SmartDashboard.putNumber("Tune Coral Pivot kI", CoralClawSettings.CoralPivotPID.getI());
        SmartDashboard.putNumber("Tune Coral Pivot kD", CoralClawSettings.CoralPivotPID.getD());

        stageChangeButtonTimer = new ElapsedTime(Resolution.MILLISECONDS);

        frameTime = runTime.time();
        runTime.reset();
    }
    
    public void setElevatorPos(double pos){
        TargetElevatorHeight = pos;
    }

    
    public void update() {
        //if(testEle!=null){ elevator.set(-testEle.getAsDouble()); }

        // limits
        TargetElevatorHeight = Functions.minMaxValue(CoralSystemSettings.minHeight, CoralSystemSettings.maxHeight, TargetElevatorHeight);

        //Calculate error
        ElevatorError=TargetElevatorHeight-CurrentElevatorHeight.getAsDouble();
        
        //Move motor
        ElevatorPID.setTarget(TargetElevatorHeight);
        if (enabled) ElevatorPower = ElevatorPID.getPower();
        else ElevatorPower = 0;

        elevatorR.set(ElevatorPower);
        elevatorL.set(-ElevatorPower);

        if (!GoToPosition) {

            /* 
            if (coralClawReference.combinedElevatorCoralPitchControl(ManualControlAxis.getAsDouble(), 
                    (TargetElevatorHeight == CoralSystemSettings.minHeight) ? CoralSystemPresets.GroundIntake.clawPitch : Math.toRadians(20), // min angle is 0 unless elevator is at the bottom
                    (TargetElevatorHeight == CoralSystemSettings.maxHeight) ? CoralClawSettings.maxPitch : Math.toRadians(75) // max angle is 75 unless elevator is at the top
            )) { // ^ this returns true if the diffy arm is already at its min or max angle and the elevator can move instead
                if (Math.abs(ManualControlAxis.getAsDouble()) > overrideOverrideTolerance && overrideManualControl) overrideManualControl = false;
                if (!overrideManualControl) TargetElevatorHeight += ManualControlAxis.getAsDouble() * CoralSystemSettings.manualControlSpeed * frameTime;
            }
                */

            if (Math.abs(ManualControlAxis.getAsDouble()) > overrideOverrideTolerance && overrideManualControl) overrideManualControl = false;
            if (!overrideManualControl) TargetElevatorHeight += ManualControlAxis.getAsDouble() * CoralSystemSettings.manualControlSpeed * frameTime;
        } else {
            if (Math.abs(ManualControlAxis.getAsDouble() - lastManualControl) > overrideOverrideTolerance && overrideManualControl) overrideManualControl = false;
            if (!overrideManualControl) TargetElevatorHeight = ManualControlAxis.getAsDouble() * (CoralSystemSettings.maxHeight - CoralSystemSettings.minHeight) + CoralSystemSettings.minHeight;
        }

        /* 
        if (overrideManualControl) {
            switch (ElevatorStage) {
                case 0: goToPreset(CoralSystemPresets.GroundIntake); break;
                case 1: goToPreset(CoralSystemPresets.L1Reef); break;
                case 2: goToPreset(CoralSystemPresets.CoralStationIntake); break;
                case 3: goToPreset(CoralSystemPresets.L2Reef); break;
                case 4: goToPreset(CoralSystemPresets.L3Reef); break;
                case 5: goToPreset(CoralSystemPresets.L4Reef); break;
            }
        }
            */
        
        
        if (pivotEnabled) {

            if (TargetArmAngle >= CoralSystemSettings.lowerLiftRange && TargetArmAngle <= CoralSystemSettings.upperLiftRange && TargetElevatorHeight < CoralSystemSettings.liftRangeHeight && CurrentElevatorHeight.getAsDouble() < CoralSystemSettings.liftRangeHeight - 2) {
                if (Math.abs(TargetArmAngle - CoralSystemSettings.lowerLiftRange) < Math.abs(TargetArmAngle - CoralSystemSettings.upperLiftRange)) TargetArmAngle = CoralSystemSettings.lowerLiftRange;
                else TargetArmAngle = CoralSystemSettings.upperLiftRange;

                TargetElevatorHeight = CoralSystemSettings.liftRangeHeight;
            }


            if (Math.abs(ManualPivotControl.getAsDouble()) >= 0.05) TargetArmAngle += CoralClawSettings.manualPivotSpeed * ManualPivotControl.getAsDouble() * frameTime;
            PivotPower = CoralClawSettings.CoralPivotPID.calculate(CurrentArmAngle.getAsDouble(), TargetArmAngle);
        } else {
            PivotPower = 0;
            TargetArmAngle = CurrentArmAngle.getAsDouble();
        }

        elevatorPivot.set(PivotPower);
        
        TargetArmAngle = Functions.minMaxValue(CoralClawSettings.minAngle, CoralClawSettings.maxAngle, TargetArmAngle);

    }


    @Override
    public void periodic() {
        frameTime = runTime.time();
        runTime.reset();
        
        //Smart Dashboard updates
        SmartDashboard.putNumber("ElevatorHeight", CurrentElevatorHeight.getAsDouble());
        SmartDashboard.putNumber("TargetEle", TargetElevatorHeight);
        SmartDashboard.putNumber("PowertoElevator",  ElevatorPower);
        SmartDashboard.putNumber("Coral Pivot Power", PivotPower);
        // SmartDashboard.putString("test", testString);//hahahaahahaha I AM DEFINITLY OKAY RIGHT NOW

        SmartDashboard.putNumber("Coral Pivot Angle Actual", elevatorPivot.getPosition().getValueAsDouble()); 


        CoralClawSettings.CoralPivotPID.setP(SmartDashboard.getNumber("Tune Coral Pivot kP", CoralClawSettings.CoralPivotPID.getP()));
        CoralClawSettings.CoralPivotPID.setI(SmartDashboard.getNumber("Tune Coral Pivot kI", CoralClawSettings.CoralPivotPID.getI()));
        CoralClawSettings.CoralPivotPID.setD(SmartDashboard.getNumber("Tune Coral Pivot kD", CoralClawSettings.CoralPivotPID.getD()));

        SmartDashboard.putNumber("Target Pivot Angle", Math.toDegrees(TargetArmAngle));
        SmartDashboard.putNumber("Current Pivot Angle", Math.toDegrees(CurrentArmAngle.getAsDouble()));
        
        //update ff
        CoralSystemSettings.ElevatorReferencePID.kP = SmartDashboard.getNumber("ElevatorPID kP", CoralSystemSettings.ElevatorReferencePID.kP);
        CoralSystemSettings.ElevatorReferencePID.kD = SmartDashboard.getNumber("ElevatorPID kD", CoralSystemSettings.ElevatorReferencePID.kD);
        CoralSystemSettings.ElevatorReferencePID.maxPosition = SmartDashboard.getNumber("ElevatorPID max pos", CoralSystemSettings.ElevatorReferencePID.maxPosition);
        CoralSystemSettings.ElevatorReferencePID.maxPower = SmartDashboard.getNumber("ElevatorPID max power", CoralSystemSettings.ElevatorReferencePID.maxPower);
        CoralSystemSettings.ElevatorReferencePID.maxSpeed = SmartDashboard.getNumber("ElevatorPID max speed", CoralSystemSettings.ElevatorReferencePID.maxSpeed);
        CoralSystemSettings.ElevatorReferencePID.maxAcceleration = SmartDashboard.getNumber("ElevatorPID max accel", CoralSystemSettings.ElevatorReferencePID.maxAcceleration);
        CoralSystemSettings.ElevatorReferencePID.maxDeceleration = SmartDashboard.getNumber("ElevatorPID max decel", CoralSystemSettings.ElevatorReferencePID.maxDeceleration);
        ElevatorPID.setSettingsTheSameAs(CoralSystemSettings.ElevatorReferencePID);
        
    }
    public boolean atPosition(){
        return ElevatorPID.closeEnough();
    }
    public int getStage(){
        return ElevatorStage;
    }
    public void setStage(int newStage){
        ElevatorStage = newStage;

        overrideManualControl = true;
        if (GoToPosition) lastManualControl = ManualControlAxis.getAsDouble();
    }

    public void upOneStage() {
        stageChangeButtonTimer.reset();

        pendingStageChange = true;
        lastWasStagingUp = true;

        ElevatorStage += 1;
        if (ElevatorStage > numberOfStages) ElevatorStage = numberOfStages;

    }


    public void downOneStage() {
        stageChangeButtonTimer.reset();

        pendingStageChange = true;
        lastWasStagingUp = false;

        ElevatorStage -= 1;
        if (ElevatorStage < 0) ElevatorStage = 0;

        
    }

    public void stopChangingStage() {
        
        if (stageChangeButtonTimer.time() > CoralSystemSettings.delayBeforeStaging) {
            if (lastWasStagingUp) ElevatorStage = numberOfStages;
            else ElevatorStage = 0;
        }

        overrideManualControl = true;
        if (GoToPosition) lastManualControl = ManualControlAxis.getAsDouble();
    }

    

    public void goToPreset(CoralSystemPreset coralSystemPreset) {
        setElevatorPos(coralSystemPreset.eleHeight);

        overrideManualControl = true;
        if (GoToPosition) lastManualControl = ManualControlAxis.getAsDouble();
    }

    public void setManualControl(DoubleSupplier controlAxis, boolean goToPosition) {
        ManualControlAxis = controlAxis;
        GoToPosition = goToPosition;
    }

    public void setManualControl(DoubleSupplier controlAxis) {
        setManualControl(controlAxis, false);
    }

    public void setManualPivotControl(DoubleSupplier controlAxis) {
        ManualPivotControl = controlAxis;
    }

    public void Enable() { enabled = true; }
    public void Disable() { enabled = false; }
    public void ToggleEnabled() { enabled = !enabled; }

    public void ArmEnable() { pivotEnabled = true; }
    public void ArmDisable() { pivotEnabled = false; }
    public void ArmToggleEnabled() { pivotEnabled = !pivotEnabled; }


    public void JumpElevatorToPickup() {

        if (TargetArmAngle < Math.toRadians(-83) && TargetArmAngle > Math.toRadians(-97)) {
            ElevatorPID.jumpSetTarget(10);
            TargetElevatorHeight = 10;
        }
    }

    public void goToFunnel() { goToPreset(CoralSystemPresets.FunnelIntake); }
    public void goToL4() { goToPreset(CoralSystemPresets.L4Reef); }
    public void goToL3() { goToPreset(CoralSystemPresets.L4Reef); }
    public void goToL2() { goToPreset(CoralSystemPresets.L2Reef); }
    public void goToStow() { goToPreset(CoralSystemPresets.Stowed); }


}
