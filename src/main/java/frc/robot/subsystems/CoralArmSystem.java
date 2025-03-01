package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings.CoralSystemSettings;
import frc.robot.Constants.MotorConstants;

public class CoralArmSystem extends SubsystemBase {
    private int ElevatorStage;

    private double TargetElevatorHeight, TargetArmAngle, ElevatorError;
    private double CurrentElevatorHeight, CurrentArmAngle, ArmError;
    private double ArmOffset = 0;

    private String testString;

    private ElevatorFeedforward elevatorFeedForward;
    private ArmFeedforward armFeedforward;

    private TalonFX leftElevator;
    private TalonFX rightElevator;
    private TalonFX coralPivot;

    private boolean enabled = true, stowCoralArm = true;

    //temp
    private DoubleSupplier testEle;


    public CoralArmSystem(String s) {
        leftElevator = new TalonFX(MotorConstants.coralLeftElevatorID);
        rightElevator = new TalonFX(MotorConstants.coralRightElevatorID);
        coralPivot = new TalonFX(MotorConstants.coralPivotID);
        TargetElevatorHeight = 0.0;
        elevatorFeedForward = new ElevatorFeedforward(CoralSystemSettings.kSE, CoralSystemSettings.kGE, CoralSystemSettings.kVE);
        armFeedforward = new ArmFeedforward(CoralSystemSettings.kSA, CoralSystemSettings.kGA, CoralSystemSettings.kVA);
    
        coralPivot.setNeutralMode(NeutralModeValue.Brake);

        testString = s;

        //put numbers so we can grab latter brr
        SmartDashboard.putNumber("Arm kSE", CoralSystemSettings.kSA);
        SmartDashboard.putNumber("Arm kGE", CoralSystemSettings.kGA);
        SmartDashboard.putNumber("Arm kVE", CoralSystemSettings.kVA);
        SmartDashboard.putNumber("TargetAngle", 0);
    }
    public CoralArmSystem(DoubleSupplier eleControl) {
        leftElevator = new TalonFX(MotorConstants.coralLeftElevatorID);
        rightElevator = new TalonFX(MotorConstants.coralRightElevatorID);
        coralPivot = new TalonFX(MotorConstants.coralPivotID);
        coralPivot.setNeutralMode(NeutralModeValue.Brake);
        testEle=eleControl;

        elevatorFeedForward = new ElevatorFeedforward(CoralSystemSettings.kSE, CoralSystemSettings.kGE, CoralSystemSettings.kVE);
        armFeedforward = new ArmFeedforward(CoralSystemSettings.kSA, CoralSystemSettings.kGA, CoralSystemSettings.kVA);
    }
    public void setElevatorPos(double pos){
        TargetElevatorHeight = pos;
    }
    public void setArmAngle(double angle){
        TargetArmAngle = angle;
    }

    
    public void update() {
        
        if(testEle!=null){
            leftElevator.set(-testEle.getAsDouble());
            rightElevator.set(testEle.getAsDouble());
        }
      
        //Update elevator position
        if (leftElevator.getPosition().getValueAsDouble()<0) 
            CurrentElevatorHeight = 0;
        else CurrentElevatorHeight = leftElevator.getPosition().getValueAsDouble(); //add offset later
        //update arm position 
        if (CurrentArmAngle<0) 
            ArmOffset= ArmOffset - CurrentArmAngle;

        CurrentArmAngle = -coralPivot.getPosition().getValueAsDouble()*(1.0/25)*(360) + ArmOffset;//make a constant latter

        //Calculate error
        ElevatorError=TargetElevatorHeight-CurrentElevatorHeight;
        ArmError = TargetArmAngle-CurrentArmAngle;
        
        //Move motors
        if(Math.abs(ElevatorError)<CoralSystemSettings.elevatorTolerance) {
            leftElevator.setVoltage(elevatorFeedForward.calculate(0));
            rightElevator.setVoltage(-elevatorFeedForward.calculate(0));
        } else{
            leftElevator.setVoltage(elevatorFeedForward.calculate(ElevatorError/CoralSystemSettings.elevatorSpeedControl));
            rightElevator.setVoltage(-elevatorFeedForward.calculate(ElevatorError/CoralSystemSettings.elevatorSpeedControl));
        }
        
        /*if(Math.abs(ArmError)<10) {
            coralPivot.setVoltage(-armFeedforward.calculate(TargetArmAngle, 0));
        }
        else*/
        if (stowCoralArm && enabled) {
            coralPivot.set(-armFeedforward.calculate(Math.toRadians(CoralSystemSettings.coralClawStowedAngle), ArmError/9));
        } else if (enabled) {
            coralPivot.set(-armFeedforward.calculate(Math.toRadians(TargetArmAngle), ArmError/9));
        } else {
            coralPivot.set(0);
        }
        

    }


    @Override
    public void periodic() {
       // TargetArmAngle= SmartDashboard.getNumber("TargetAngle", CurrentArmAngle);
        
        //Smart Dashboard updates
        SmartDashboard.putNumber("Elevator", CurrentElevatorHeight);
        SmartDashboard.putNumber("Arm Angle", CurrentArmAngle);
        SmartDashboard.putNumber("ACtual Arm Angle", coralPivot.getPosition().getValueAsDouble()*(1.0/25)*(360));
        SmartDashboard.putNumber("Target", TargetArmAngle);
        SmartDashboard.putNumber("TargetEle", TargetElevatorHeight);
        SmartDashboard.putNumber("PowertoElevator",  elevatorFeedForward.calculate(ElevatorError/CoralSystemSettings.elevatorSpeedControl));
        SmartDashboard.putString( "actual values arm", ""+elevatorFeedForward.getKs()+" "+elevatorFeedForward.getKg()+" "+elevatorFeedForward.getKv());
       // SmartDashboard.putString("test", testString);//hahahaahahaha I AM DEFINITLY OKAY RIGHT NOW
        SmartDashboard.putNumber("Left ElevatorAct ", leftElevator.getPosition().getValueAsDouble());

        //update ff
        boolean check1 = armFeedforward.getKs()!=SmartDashboard.getNumber("Arm kSE", CoralSystemSettings.kSE);
        boolean check2 = armFeedforward.getKg()!=SmartDashboard.getNumber("Arm kGE", CoralSystemSettings.kGE);
        boolean check3 = armFeedforward.getKv()!=SmartDashboard.getNumber("Arm kVE", CoralSystemSettings.kVE);
        if(check1 || check2 || check3){
            SmartDashboard.putString( "stuff", "e");
            armFeedforward = new ArmFeedforward(
            SmartDashboard.getNumber("Arm kSE", CoralSystemSettings.kSE), 
            SmartDashboard.getNumber("Arm kGE", CoralSystemSettings.kGE), 
            SmartDashboard.getNumber("Arm kVE", CoralSystemSettings.kVE));
        }
        

        
    }
    public boolean atPosition(){
        if(Math.abs(ElevatorError)<CoralSystemSettings.elevatorTolerance)
        return true;
        return false;
    }
    public int getStage(){
        return ElevatorStage;
    }
    public void setStage(int newStage){
        ElevatorStage = newStage;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void toggleCoralArm() {
        enabled = !enabled;
    }

    public void toggleStoreArm() {
        stowCoralArm = !stowCoralArm;
    }

    public void unstowArm() {
        stowCoralArm = false;
    }


}
