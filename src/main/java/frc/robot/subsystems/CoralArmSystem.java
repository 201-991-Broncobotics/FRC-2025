package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.robot.Constants.MotorConstants;

public class CoralArmSystem extends SubsystemBase {

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
        elevatorFeedForward = new ElevatorFeedforward(Settings.CoralSystemSettings.kSE, Settings.CoralSystemSettings.kGE, Settings.CoralSystemSettings.kVE, 0);
        armFeedforward = new ArmFeedforward(Settings.CoralSystemSettings.kSA, Settings.CoralSystemSettings.kGA, Settings.CoralSystemSettings.kVA, 0);

        testString = s;

        //put numbers so we can grab latter brr
        SmartDashboard.putNumber("Elevator kSE", Settings.CoralSystemSettings.kSE);
        SmartDashboard.putNumber("Elevator kGE", Settings.CoralSystemSettings.kGE);
        SmartDashboard.putNumber("Elevator kVE", Settings.CoralSystemSettings.kVE);
    }
    public CoralArmSystem(DoubleSupplier eleControl) {
        leftElevator = new TalonFX(MotorConstants.coralLeftElevatorID);
        rightElevator = new TalonFX(MotorConstants.coralRightElevatorID);
        coralPivot = new TalonFX(MotorConstants.coralPivotID);
        testEle=eleControl;

        elevatorFeedForward = new ElevatorFeedforward(Settings.CoralSystemSettings.kSE, Settings.CoralSystemSettings.kGE, Settings.CoralSystemSettings.kVE, 0);
        armFeedforward = new ArmFeedforward(Settings.CoralSystemSettings.kSA, Settings.CoralSystemSettings.kGA, Settings.CoralSystemSettings.kVA, 0);
    }
    public void setElevatorPos(double pos){
        TargetElevatorHeight = pos;
    }
    public void setArmAngle(double angle){
        TargetArmAngle = angle;
    }


    @Override
    public void periodic() {
        if(testEle!=null){
            leftElevator.set(-testEle.getAsDouble());
            rightElevator.set(testEle.getAsDouble());
        }
        //update arm rotation
        CurrentArmAngle=coralPivot.getPosition().getValueAsDouble();
        //Update elevator position
        if (leftElevator.getPosition().getValueAsDouble()<0) 
            CurrentElevatorHeight = 0;
        else CurrentElevatorHeight = leftElevator.getPosition().getValueAsDouble(); //add offset later

        //Calculate error
        ElevatorError=TargetElevatorHeight-CurrentElevatorHeight;
        //Move motors
        if(Math.abs(ElevatorError)<Settings.CoralSystemSettings.elevatorTolerance) {
            leftElevator.set(elevatorFeedForward.calculate(0));
            rightElevator.set(-elevatorFeedForward.calculate(0));
        }
        else{
            leftElevator.set(elevatorFeedForward.calculate(ElevatorError/Settings.CoralSystemSettings.elevatorSpeedControl));
            rightElevator.set(-elevatorFeedForward.calculate(ElevatorError/Settings.CoralSystemSettings.elevatorSpeedControl));
        }
        
        //Smart Dashboard updates
        SmartDashboard.putNumber("Elevator", CurrentElevatorHeight);
        SmartDashboard.putNumber("Arm Angle", CurrentArmAngle);
        SmartDashboard.putNumber("Target", TargetElevatorHeight);
        SmartDashboard.putNumber("Power",  elevatorFeedForward.calculate(ElevatorError/Settings.CoralSystemSettings.elevatorSpeedControl));
        SmartDashboard.putString( "actual vals", ""+elevatorFeedForward.getKs()+" "+elevatorFeedForward.getKg()+" "+elevatorFeedForward.getKv());
       // SmartDashboard.putString("test", testString);//hahahaahahaha I AM DEFINITLY OKAY RIGHT NOW
        SmartDashboard.putNumber("Left ElevatorAct ", leftElevator.getPosition().getValueAsDouble());

        //update ff
        boolean check1 = elevatorFeedForward.getKs()!=SmartDashboard.getNumber("Elevator kSE", Settings.CoralSystemSettings.kSE);
        boolean check2 = elevatorFeedForward.getKs()!=SmartDashboard.getNumber("Elevator kGE", Settings.CoralSystemSettings.kGE);
        boolean check3 = elevatorFeedForward.getKs()!=SmartDashboard.getNumber("Elevator kVE", Settings.CoralSystemSettings.kVE);
        if(check1 || check2 || check3){
            SmartDashboard.putString( "stuff", "a");
            elevatorFeedForward = new ElevatorFeedforward(
            SmartDashboard.getNumber("Elevator kSE", Settings.CoralSystemSettings.kSE), 
            SmartDashboard.getNumber("Elevator kGE", Settings.CoralSystemSettings.kGE), 
            SmartDashboard.getNumber("Elevator kVE", Settings.CoralSystemSettings.kVE));
        }
        

        
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
