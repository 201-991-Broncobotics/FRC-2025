package frc.robot.subsystems;

import static frc.robot.Settings.CoralClawSettings.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.MotorConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings.CoralClawSettings;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

public class CoralClaw extends SubsystemBase {

    //Targeted Pos for each motor
    double rMotorPos = 0.0; //Current Diffy Position for Right motor
    double lMotorPos = 0.0; //Current Diffy Position for Left Motor

    double Roll, Pitch;

    private double RollerPower;

    private SparkMax rmotor, lmotor, rollerMotor;
    private DoubleSupplier rencoder,lencoder; // gets angle in radians at the joint
    private double gearRatio = 1.0/4.0 * 1.0/5.0 * 18.0/30.0; // gear ratio from motor to each joint side

    private SparkMaxConfig rightMotorConfig, leftMotorConfig, coralMotorConfig;

    public CoralClaw() {
        RollerPower = 0;

        Roll = CoralClawSettings.startRoll;
        Pitch = CoralClawSettings.startPitch;

        // initialize motor
        rmotor = new SparkMax(rightDiffyID, MotorType.kBrushless);
        lmotor = new SparkMax(leftDiffyID, MotorType.kBrushless);
        rollerMotor = new SparkMax(MotorConstants.coralRollerID, MotorType.kBrushless);

        // create configurations
        rightMotorConfig = new SparkMaxConfig();
        leftMotorConfig = new SparkMaxConfig();
        coralMotorConfig = new SparkMaxConfig();


        // set configurations
        rightMotorConfig.idleMode(IdleMode.kBrake);
        leftMotorConfig.idleMode(IdleMode.kBrake);
        coralMotorConfig.idleMode(IdleMode.kCoast);
        rightMotorConfig.closedLoop.pidf(kP, kI, kD, RFF);
        leftMotorConfig.closedLoop.pidf(kP, kI, kD, LFF);

        // apply configurations initially
        rmotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lmotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerMotor.configure(coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Encoder object created to display position values - I think it uses internal encoder or smth?
        rencoder = () -> gearRatio * rmotor.getEncoder().getPosition() * 2*Math.PI; // converts to radians
        lencoder = () -> gearRatio * lmotor.getEncoder().getPosition() * 2*Math.PI;

        if (Settings.tuningTelemetryEnabled) {
            SmartDashboard.putNumber("Tune Coral Claw kP", CoralClawSettings.kP);
            SmartDashboard.putNumber("Tune Coral Claw kI", CoralClawSettings.kI);
            SmartDashboard.putNumber("Tune Coral Claw kD", CoralClawSettings.kD);
            SmartDashboard.putNumber("Tune Coral Claw Right FF", CoralClawSettings.RFF);
            SmartDashboard.putNumber("Tune Coral Claw Left FF", CoralClawSettings.LFF);
        }
            
    }


    public void update() {
        //Limits
        if(Pitch > maxPitch) {Pitch = maxPitch;} else if (Pitch < minPitch){Pitch = minPitch;}
        if(Roll > rollRange) {Roll = rollRange;} else if (Roll < -rollRange){Roll = -rollRange;}

        rollerMotor.set(RollerPower);

        calculateTargetPos();//get angles for each motor

        rmotor.getClosedLoopController().setReference(rMotorPos/360.0, SparkMax.ControlType.kPosition);//convert 0-360 to 0-1 & set Motor
        lmotor.getClosedLoopController().setReference(lMotorPos/360.0, SparkMax.ControlType.kPosition);//convert 0-360 to 0-1 & set Motor
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // read PID coefficients from SmartDashboard
        if (Settings.tuningTelemetryEnabled) {
            CoralClawSettings.kP = SmartDashboard.getNumber("Tune Coral Claw kP", CoralClawSettings.kP);
            CoralClawSettings.kI = SmartDashboard.getNumber("Tune Coral Claw kI", CoralClawSettings.kI);
            CoralClawSettings.kD = SmartDashboard.getNumber("Tune Coral Claw kD", CoralClawSettings.kD);
            CoralClawSettings.RFF = SmartDashboard.getNumber("Tune Coral Claw Right FF", CoralClawSettings.RFF);
            CoralClawSettings.LFF = SmartDashboard.getNumber("Tune Coral Claw Left FF", CoralClawSettings.LFF);
        }

        rightMotorConfig.closedLoop.pidf(CoralClawSettings.kP, CoralClawSettings.kI, CoralClawSettings.kD, CoralClawSettings.RFF);
        leftMotorConfig.closedLoop.pidf(CoralClawSettings.kP, CoralClawSettings.kI, CoralClawSettings.kD, CoralClawSettings.LFF);

        rmotor.configureAsync(rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.putNumber("CoralClaw Target Pitch", Pitch);
        SmartDashboard.putNumber("CoralClaw Target Roll", Pitch);
        SmartDashboard.putNumber("CoralClaw Current Pitch", getCurrentPitch());
        SmartDashboard.putNumber("CoralClaw Current Roll", getCurrentRoll());
        SmartDashboard.putNumber("CoralClaw Left Motor Power", getCurrentPitch());
        SmartDashboard.putNumber("CoralClaw Right Motor Power", getCurrentRoll());
        SmartDashboard.putNumber("CoralClaw Roller Power", RollerPower);

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void calculateTargetPos(){
        lMotorPos = Pitch-(Roll/2.0);
        rMotorPos = Pitch+(Roll/2.0);
    }

    /**
     * Gets current pitch in radians
     */
    public double getCurrentPitch() {
        return 0.5 * (rencoder.getAsDouble() + lencoder.getAsDouble());
    }
    /**
     * Gets current roll in radians
     */
    public double getCurrentRoll() {
        return rencoder.getAsDouble() - lencoder.getAsDouble();
    }

    public void stopRoller() {
        RollerPower = 0;
    }

    public void intakeRoller() {
        RollerPower = CoralClawSettings.intakePower;
    }

    public void outtakeRoller() {
        RollerPower = CoralClawSettings.outtakePower;
    }

}