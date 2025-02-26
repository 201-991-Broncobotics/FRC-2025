
package frc.robot.subsystems;

import static frc.robot.Settings.CoralClawSettings.*;
import static frc.robot.Constants.MotorConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class CoralClaw extends SubsystemBase {

    //Targeted Pos for each motor
    double rMotorPos = 0.0; //Current Diffy Position for Right motor
    double lMotorPos = 0.0; //Current Diffy Position for Left Motor

    double Roll, Pitch;

    private CANSparkMax rmotor, lmotor;
    private SparkPIDController rpidController, lpidController;
    private RelativeEncoder rencoder,lencoder;

    public CoralClaw(double startRoll, double startPitch) {
            Roll = startRoll;
            Pitch = startPitch;

            // initialize motor
            rmotor = new CANSparkMax(rightDiffyID, MotorType.kBrushless);
            lmotor = new CANSparkMax(leftDiffyID, MotorType.kBrushless);

            /**
             * The restoreFactoryDefaults method can be used to reset the configuration parameters
             * in the SPARK MAX to their factory default state. If no argument is passed, these
             * parameters will not persist between power cycles
             */
            rmotor.restoreFactoryDefaults();
            lmotor.restoreFactoryDefaults();

            /**
             * In order to use PID functionality for a controller, a SparkPIDController object
             * is constructed by calling the getPIDController() method on an existing
             * CANSparkMax object
             */
            rpidController = rmotor.getPIDController();
            lpidController = lmotor.getPIDController();

            // Encoder object created to display position values - I think it uses ionternal encoder or smth?
            rencoder = rmotor.getEncoder();
            lencoder = lmotor.getEncoder();

            rpidController.setP(RkP);
            rpidController.setI(RkI);
            rpidController.setD(RkD);
            rpidController.setIZone(0); //dunno what this is
            rpidController.setFF(0);    //dunno what this is either
            rpidController.setOutputRange(-Rspeed, Rspeed); //i think this is speed, change later to check
            lpidController.setP(RkP);
            lpidController.setI(RkI);
            lpidController.setD(RkD);
            lpidController.setIZone(0); //dunno what this is
            lpidController.setFF(0);    //dunno what this is either
            lpidController.setOutputRange(-Lspeed, Lspeed); //i think this is speed, change later to check
    }

    @Override
    public void periodic() {
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

        rpidController.setReference(rMotorPos/360, CANSparkMax.ControlType.kPosition);//convert 0-360 to 0-1 & set Moto
        lpidController.setReference(lMotorPos/360, CANSparkMax.ControlType.kPosition);//convert 0-360 to 0-1 & set Motor
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