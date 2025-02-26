package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.robot.Constants.MotorConstants;

public class CoralArmSystem extends SubsystemBase {
    private int ElevatorStage;

    private double TargetElevatorHeight, TargetArmAngle, ElevatorError;
    private double CurrentElevatorHeight, CurrentArmAngle, ArmError;

    private String testString;

    private ElevatorFeedforward elevatorFeedForward;
    private ArmFeedforward armFeedforward;

    private TalonFX leftElevator;
    private TalonFX rightElevator;
    private TalonFX coralPivot;

    //temp
    private DoubleSupplier testEle;


    public CoralArmSystem(String s) {
        leftElevator = new TalonFX(MotorConstants.coralLeftElevatorID);
        rightElevator = new TalonFX(MotorConstants.coralRightElevatorID);
        coralPivot = new TalonFX(MotorConstants.coralPivotID);
        TargetElevatorHeight = 0.0;
        elevatorFeedForward = new ElevatorFeedforward(Settings.CoralSystemSettings.kSE, Settings.CoralSystemSettings.kGE, Settings.CoralSystemSettings.kVE);
        armFeedforward = new ArmFeedforward(Settings.CoralSystemSettings.kSA, Settings.CoralSystemSettings.kGA, Settings.CoralSystemSettings.kVA);

        testString = s;

        //put numbers so we can grab latter brr
        SmartDashboard.putNumber("Arm kSE", Settings.CoralSystemSettings.kSA);
        SmartDashboard.putNumber("Arm kGE", Settings.CoralSystemSettings.kGA);
        SmartDashboard.putNumber("Arm kVE", Settings.CoralSystemSettings.kVA);
        SmartDashboard.putNumber("TargetAngle", 0);
    }
    public CoralArmSystem(DoubleSupplier eleControl) {
        leftElevator = new TalonFX(MotorConstants.coralLeftElevatorID);
        rightElevator = new TalonFX(MotorConstants.coralRightElevatorID);
        coralPivot = new TalonFX(MotorConstants.coralPivotID);
        testEle=eleControl;

        elevatorFeedForward = new ElevatorFeedforward(Settings.CoralSystemSettings.kSE, Settings.CoralSystemSettings.kGE, Settings.CoralSystemSettings.kVE);
        armFeedforward = new ArmFeedforward(Settings.CoralSystemSettings.kSA, Settings.CoralSystemSettings.kGA, Settings.CoralSystemSettings.kVA);
    }
    public void setElevatorPos(double pos){
        TargetElevatorHeight = pos;
    }
    public void setArmAngle(double angle){
        TargetArmAngle = angle;
    }


    @Override
    public void periodic() {
        TargetArmAngle= SmartDashboard.getNumber("TargetAngle", CurrentArmAngle);
        
        if(testEle!=null){
            leftElevator.set(-testEle.getAsDouble());
            rightElevator.set(testEle.getAsDouble());
        }
      
        //Update elevator position
        if (leftElevator.getPosition().getValueAsDouble()<0) 
            CurrentElevatorHeight = 0;
        else CurrentElevatorHeight = leftElevator.getPosition().getValueAsDouble(); //add offset later
        //update arm position 
        if (-coralPivot.getPosition().getValueAsDouble()<0) 
            CurrentArmAngle = 0;
        else CurrentArmAngle = -coralPivot.getPosition().getValueAsDouble()*(1.0/25)*(360);//make a constant latter

        //Calculate error
        ElevatorError=TargetElevatorHeight-CurrentElevatorHeight;
        ArmError = TargetArmAngle-CurrentArmAngle;
        
        //Move motors
        if(Math.abs(ElevatorError)<Settings.CoralSystemSettings.elevatorTolerance) {
            leftElevator.set(elevatorFeedForward.calculate(0));
            rightElevator.set(-elevatorFeedForward.calculate(0));
        }
        else{
            leftElevator.setVoltage(elevatorFeedForward.calculate(ElevatorError/Settings.CoralSystemSettings.elevatorSpeedControl));
            rightElevator.setVoltage(-elevatorFeedForward.calculate(ElevatorError/Settings.CoralSystemSettings.elevatorSpeedControl));
        }
        
        /*if(Math.abs(ArmError)<10) {
            coralPivot.set(-armFeedforward.calculate(TargetArmAngle, 0));
        }
        else*/
        {
            coralPivot.set(-armFeedforward.calculate(Math.toRadians(TargetArmAngle), ArmError/9));
        }
        
        //Smart Dashboard updates
        SmartDashboard.putNumber("Elevator", CurrentElevatorHeight);
        SmartDashboard.putNumber("Arm Angle", CurrentArmAngle);
        SmartDashboard.putNumber("Target", TargetArmAngle);
        SmartDashboard.putNumber("PowertoArm",  -armFeedforward.calculate(TargetArmAngle, ElevatorError/10));
        SmartDashboard.putString( "actual values arm", ""+armFeedforward.getKs()+" "+armFeedforward.getKg()+" "+armFeedforward.getKv());
       // SmartDashboard.putString("test", testString);//hahahaahahaha I AM DEFINITLY OKAY RIGHT NOW
        SmartDashboard.putNumber("Left ElevatorAct ", leftElevator.getPosition().getValueAsDouble());

        //update ff
        boolean check1 = armFeedforward.getKs()!=SmartDashboard.getNumber("Arm kSE", Settings.CoralSystemSettings.kSE);
        boolean check2 = armFeedforward.getKg()!=SmartDashboard.getNumber("Arm kGE", Settings.CoralSystemSettings.kGE);
        boolean check3 = armFeedforward.getKv()!=SmartDashboard.getNumber("Arm kVE", Settings.CoralSystemSettings.kVE);
        if(check1 || check2 || check3){
            SmartDashboard.putString( "stuff", "e");
            armFeedforward = new ArmFeedforward(
            SmartDashboard.getNumber("Arm kSE", Settings.CoralSystemSettings.kSE), 
            SmartDashboard.getNumber("Arm kGE", Settings.CoralSystemSettings.kGE), 
            SmartDashboard.getNumber("Arm kVE", Settings.CoralSystemSettings.kVE));
        }
        

        
    }
    public boolean atPosition(){
        if(Math.abs(ElevatorError)<Settings.CoralSystemSettings.elevatorTolerance)
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
}
