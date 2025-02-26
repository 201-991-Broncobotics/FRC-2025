package frc.robot.subsystems;

import static frc.robot.Settings.CoralClawSettings.*;
import static frc.robot.Constants.MotorConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

public class CoralClaw extends SubsystemBase {

    //Targeted Pos for each motor
    double rMotorPos = 0.0; //Current Diffy Position for Right motor
    double lMotorPos = 0.0; //Current Diffy Position for Left Motor

    double Roll, Pitch;

    private SparkMax rmotor, lmotor;
    private SparkClosedLoopController rpidController, lpidController;
    private ClosedLoopConfig rightPIDConfig, leftPIDConfig;
    private RelativeEncoder rencoder,lencoder;

    private SparkMaxConfig motorConfig;

    public CoralClaw(double startRoll, double startPitch) {
            Roll = startRoll;
            Pitch = startPitch;

            // initialize motor
            rmotor = new SparkMax(rightDiffyID, MotorType.kBrushless);
            lmotor = new SparkMax(leftDiffyID, MotorType.kBrushless);

            motorConfig = new SparkMaxConfig();
            rmotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            lmotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
            // gets closed loop controllers and applies them which is so much more complicated than it was before
            rpidController = rmotor.getClosedLoopController();
            lpidController = lmotor.getClosedLoopController();
            rightPIDConfig = new ClosedLoopConfig();
            leftPIDConfig = new ClosedLoopConfig();

            rightPIDConfig.pidf(RkP, RkI, RkD, RFF);
            leftPIDConfig.pidf(RkP, RkI, RkD, LFF);

            // Encoder object created to display position values - I think it uses ionternal encoder or smth?
            //rencoder = rmotor.getEncoder();
            //lencoder = lmotor.getEncoder();

            //rmotor.
    }

    @Override
    public void periodic() {
        /* 

        // This method will be called once per scheduler run
        // read PID coefficients from SmartDashboard
        double rkP = SmartDashboard.getNumber("Right Claw Motor P Gain", 0);
        double rkI = SmartDashboard.getNumber("Right Claw Motor I Gain", 0);
        double rkD = SmartDashboard.getNumber("Right Claw Motor D Gain", 0);
        double lkP = SmartDashboard.getNumber("Left Claw Motor P Gain", 0);
        double lkI = SmartDashboard.getNumber("Left Claw Motor I Gain", 0);
        double lkD = SmartDashboard.getNumber("Left Claw Motor D Gain", 0);
        double rSpeed = SmartDashboard.getNumber("Right Claw Motor Speed", 0);
        double lSpeed = SmartDashboard.getNumber("Left Claw Motor Speed", 0);


        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((rkP != RkP)) { rpidController.setP(rkP); RkP = rkP; }
        if((rkI != RkI)) { rpidController.setI(rkI); RkI = rkI; }
        if((rkD != RkD)) { rpidController.setD(rkD); RkD = rkD; }
        if((lkP != LkP)) { rpidController.setP(lkP); LkP = lkP; }
        if((lkI != LkI)) { rpidController.setI(lkI); LkI = lkI; }
        if((lkD != LkD)) { rpidController.setD(lkD); LkD = lkD; }
        //Like i said, i think this is speed but im not touching it (yet)
        if((rSpeed != Rspeed)) {
            rpidController.setOutputRange(-rSpeed, rSpeed);
            Rspeed = rSpeed;
        }
        if((lSpeed != Lspeed)) {
            lpidController.setOutputRange(-lSpeed, lSpeed);
            Lspeed = lSpeed;
        }

        //Limits
        if(Pitch > maxPitch){Pitch = maxPitch;} else if (Pitch < minPitch){Pitch = minPitch;}
        if(Roll > rollRange){Roll = rollRange;} else if (Roll < -rollRange){Roll = -rollRange;}

        calculateTargetPos();//get angles for each motor

        rpidController.setReference(rMotorPos/360, SparkMax.ControlType.kPosition);//convert 0-360 to 0-1 & set Moto
        lpidController.setReference(lMotorPos/360, SparkMax.ControlType.kPosition);//convert 0-360 to 0-1 & set Motor

        */
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void calculateTargetPos(){
        lMotorPos = Pitch-(Roll/2);
        rMotorPos = Pitch+(Roll/2);
    }
}