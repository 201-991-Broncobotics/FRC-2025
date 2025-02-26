package frc.robot.subsystems;

import static frc.robot.utility.Functions.round;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeArmConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings;
import frc.robot.Settings.AlgaeArmSettings;
import frc.robot.Settings.AlgaeRollerSettings;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.Functions;
import frc.robot.utility.MotionProfile2d;
import frc.robot.utility.Vector2d;


public class AlgaeArmSystem extends SubsystemBase {

    /*                                     -   +                                 From Lower Joint (default)
        +                          - - - - - 0 + + + + + + + + + + + + + + + + + + + + + + + + +  
        |                  %/@(                                                                 |
        |                 &@%%                                                                  |
        |                 ,,,& & @                                                              |
        |                 ,,@&//%%                                                              |
        |                ,,, @%# ¯}                       #,(                                   |
        |                ,,  @%%                        @*O,&\____________..±±_____%#//]        |
        |               ,,, @/@                        #,,,%//____________.¶#&@@==== §          |
        |               ,,@&  /                      %,,,&                  &|     %#//]        |
        |              @,,  / (  &&                @%,,(                                        |
        |              ,,(((/(@  (&               &,,,#                                         |
        |             ,,, (((@   ,,             #,,,@                                           |
        |             ,,, @@C    ,,%          .@,,@/                                            |
        |            @,,&((@(    %%@         @,,,%                                Angles        | +
        |            ,,,(/@@     %%@       &#O,@                                     90         0
        |           @,,     ,.....,&....   &/&(/{}]                  Upper J:    180 + 0        | -
        |          @#,,@    ,....  &...@    ,,, ¯¯                                  -90         |
        |          .&((L_   ,....  @..%/#@&@@@@                                                    
        |         /(((*(((\ &##%(  @(##&@@@#@@@(%%                                   90               
        |       /((/@%&%@((\@,.(*  @(.(,&&%&%#%%%##@                 Lower J:    180 + 0                    
        |  [¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯C)¯C)¯|¯¯¯]                           -90      
       +|  [__________________________________/__/__1___]       
        0       '¯'               0              '¯'          
       -| - - - - - - - - - - - - ^ + + + + + + + + + + + + + + + + + + + + + + 
        |                     From Robot
     */

    public boolean RightBias; // Default is based on starting config. This determines which side the middle joint will always try to stay on 
    // relative to a line between the lower joint and the target. This is because there are almost always two solutions to the kinematics

    private double CurrentL1Angle, CurrentL2Angle; // radians
    private Vector2d MovingTarget; // in inches and (0, 0) is at the lower joint
    public Vector2d Target;

    private DoubleSupplier L1Encoder, L2Encoder;

    private MotionProfile2d motionProfile;

    private double L1Offset, L2Offset; 

    private ElapsedTime frameTimer;
    private double frameTime = 0.01;

    private TalonFX bottomPivot;
    private TalonFX topPivot;
    private SparkMax clawRoller;

    private SparkMaxConfig clawRollerConfig;


    private double lowerGearRatio = (1.0/5.0 * 1.0/5.0) * 2*Math.PI; // motor angle * gear ratio = actual angle
    private double upperGearRatio = (1.0/5.0 * 1.0/3.0 * 1.0/3.0) * 2*Math.PI;

    private double clawRollerPower = 0;
    private boolean armStopped = false;

    private boolean continueSpinningRoller = false;

    /**
     * The 0 angles for each joint straight forward so that pi/2 (90 degrees) is straight up
     */
    public AlgaeArmSystem(double firstSegmentStartAngle, double secondSegmentStartAngle) {

        // initialize motors
        bottomPivot = new TalonFX(MotorConstants.algaeBottomPivotID);
        topPivot = new TalonFX(MotorConstants.algaeTopPivotID);
        clawRoller = new SparkMax(MotorConstants.algaeRollerID, MotorType.kBrushless);
        clawRollerConfig.idleMode(IdleMode.kCoast);

        bottomPivot.setNeutralMode(NeutralModeValue.Brake);
        topPivot.setNeutralMode(NeutralModeValue.Brake);
        clawRoller.configure(clawRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        L1Offset = firstSegmentStartAngle - bottomPivot.getPosition().getValueAsDouble() * lowerGearRatio;
        L2Offset = secondSegmentStartAngle - topPivot.getPosition().getValueAsDouble() * upperGearRatio;
        // initialize encoder suppliers
        L1Encoder = () -> bottomPivot.getPosition().getValueAsDouble() * lowerGearRatio + L1Offset; //I think this should get the currect position but will need testing
        L2Encoder = () -> topPivot.getPosition().getValueAsDouble() * upperGearRatio + L2Offset - L1Encoder.getAsDouble() + firstSegmentStartAngle; // this last part is because I didn't understand the mechanism exactly which really complicated kinematics 

        if (Functions.normalizeAngle(L1Encoder.getAsDouble() - L2Encoder.getAsDouble()) >= 0) RightBias = false;
        else RightBias = true;

        CurrentL1Angle = L1Encoder.getAsDouble();
        CurrentL2Angle = L2Encoder.getAsDouble();

        // Get start position
        MovingTarget = getCurrentPoint();
        Target = MovingTarget;

        motionProfile = new MotionProfile2d(Target, AlgaeArmSettings.maxAcceleration, AlgaeArmSettings.maxDeceleration, AlgaeArmSettings.maxSpeed);

        frameTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        continueSpinningRoller = false;

        // Place initial tuning values into smartdashboard

        if (Settings.tuningTelemetryEnabled) {
            SmartDashboard.putNumber("Tune Algae Lower kP", AlgaeArmSettings.LowerJointPID.getP());
            SmartDashboard.putNumber("Tune Algae Lower kI", AlgaeArmSettings.LowerJointPID.getI());
            SmartDashboard.putNumber("Tune Algae Lower kD", AlgaeArmSettings.LowerJointPID.getD());
            SmartDashboard.putNumber("Tune Algae Upper kP", AlgaeArmSettings.UpperJointPID.getP());
            SmartDashboard.putNumber("Tune Algae Upper kI", AlgaeArmSettings.UpperJointPID.getI());
            SmartDashboard.putNumber("Tune Algae Upper kD", AlgaeArmSettings.UpperJointPID.getD());

            SmartDashboard.putNumber("Tune Algae max Accel", AlgaeArmSettings.maxAcceleration);
            SmartDashboard.putNumber("Tune Algae max Decel", AlgaeArmSettings.maxDeceleration);
            SmartDashboard.putNumber("Tune Algae max Speed", AlgaeArmSettings.maxSpeed);

            SmartDashboard.putNumber("Tune Algae lower start angle", AlgaeArmSettings.AlgaeArmLowerJointStartAngle);
            SmartDashboard.putNumber("Tune Algae upper start angle", AlgaeArmSettings.AlgaeArmUpperJointStartAngle);

            SmartDashboard.putNumber("Tune Algae voltage tolerance", AlgaeArmSettings.voltageTolerance);

            SmartDashboard.putBoolean("Tune Algae use feedforward", AlgaeArmSettings.useFeedforward);
            SmartDashboard.putNumber("Tune Algae lower joint kS", AlgaeArmSettings.L1Feedforward.getKs());
            SmartDashboard.putNumber("Tune Algae lower joint kG", AlgaeArmSettings.L1Feedforward.getKg());
            SmartDashboard.putNumber("Tune Algae lower joint kV", AlgaeArmSettings.L1Feedforward.getKv());
            SmartDashboard.putNumber("Tune Algae upper joint kS", AlgaeArmSettings.L2Feedforward.getKs());
            SmartDashboard.putNumber("Tune Algae upper joint kG", AlgaeArmSettings.L2Feedforward.getKg());
            SmartDashboard.putNumber("Tune Algae upper joint kV", AlgaeArmSettings.L2Feedforward.getKv());

            //SmartDashboard.putNumber("Tune Algae Limit maxAngleLowerJoint", AlgaeArmSettings.maxAngleLowerJoint);
            //SmartDashboard.putNumber("Tune Algae Limit minAngleLowerJoint", AlgaeArmSettings.minAngleLowerJoint);
            //SmartDashboard.putNumber("Tune Algae Limit maxAngleUpperJointFromLower", AlgaeArmSettings.maxAngleUpperJointFromLower);
            //SmartDashboard.putNumber("Tune Algae Limit minAngleUpperJointFromLower", AlgaeArmSettings.minAngleUpperJointFromLower);
            //SmartDashboard.putNumber("Tune Algae Limit maxDistanceInX", AlgaeArmSettings.maxDistanceInX);
            //SmartDashboard.putNumber("Tune Algae Limit maxDistanceOutX", AlgaeArmSettings.maxDistanceOutX);
            //SmartDashboard.putNumber("Tune Algae Limit maxDistanceDownY", AlgaeArmSettings.maxDistanceDownY);

            SmartDashboard.putNumber("Tune Algae Manual Control Speed", AlgaeArmSettings.maxJoystickMovementSpeed);
        }
    }




    @Override
    public void periodic() { // Main run method
        frameTime = frameTimer.time();
        frameTimer.reset();

        //update settings
        if (Settings.tuningTelemetryEnabled) {
            AlgaeArmSettings.LowerJointPID.setP(SmartDashboard.getNumber("Tune Algae Lower kP", AlgaeArmSettings.LowerJointPID.getP()));
            AlgaeArmSettings.LowerJointPID.setI(SmartDashboard.getNumber("Tune Algae Lower kI", AlgaeArmSettings.LowerJointPID.getI()));
            AlgaeArmSettings.LowerJointPID.setD(SmartDashboard.getNumber("Tune Algae Lower kD", AlgaeArmSettings.LowerJointPID.getD()));
            AlgaeArmSettings.UpperJointPID.setP(SmartDashboard.getNumber("Tune Algae Upper kP", AlgaeArmSettings.UpperJointPID.getP()));
            AlgaeArmSettings.UpperJointPID.setI(SmartDashboard.getNumber("Tune Algae Upper kI", AlgaeArmSettings.UpperJointPID.getI()));
            AlgaeArmSettings.UpperJointPID.setD(SmartDashboard.getNumber("Tune Algae Upper kD", AlgaeArmSettings.UpperJointPID.getD()));

            AlgaeArmSettings.maxAcceleration = SmartDashboard.getNumber("Tune Algae max Accel", AlgaeArmSettings.maxAcceleration);
            AlgaeArmSettings.maxDeceleration = SmartDashboard.getNumber("Tune Algae max Decel", AlgaeArmSettings.maxDeceleration);
            AlgaeArmSettings.maxSpeed = SmartDashboard.getNumber("Tune Algae max Speed", AlgaeArmSettings.maxSpeed);
            motionProfile.updateSettings(AlgaeArmSettings.maxAcceleration, AlgaeArmSettings.maxDeceleration, AlgaeArmSettings.maxSpeed);

            AlgaeArmSettings.AlgaeArmLowerJointStartAngle = SmartDashboard.getNumber("Tune Algae lower start angle", AlgaeArmSettings.AlgaeArmLowerJointStartAngle);
            AlgaeArmSettings.AlgaeArmUpperJointStartAngle = SmartDashboard.getNumber("Tune Algae upper start angle", AlgaeArmSettings.AlgaeArmUpperJointStartAngle);

            AlgaeArmSettings.voltageTolerance = SmartDashboard.getNumber("Tune Algae voltage tolerance", AlgaeArmSettings.voltageTolerance);

            AlgaeArmSettings.useFeedforward = SmartDashboard.getBoolean("Tune Algae use feedforward", AlgaeArmSettings.useFeedforward);
            double lowerJointKS = SmartDashboard.getNumber("Tune Algae lower joint kS", AlgaeArmSettings.L1Feedforward.getKs());
            double lowerJointKG = SmartDashboard.getNumber("Tune Algae lower joint kG", AlgaeArmSettings.L1Feedforward.getKg());
            double lowerJointKV = SmartDashboard.getNumber("Tune Algae lower joint kV", AlgaeArmSettings.L1Feedforward.getKv());
            double upperJointKS = SmartDashboard.getNumber("Tune Algae upper joint kS", AlgaeArmSettings.L2Feedforward.getKs());
            double upperJointKG = SmartDashboard.getNumber("Tune Algae upper joint kG", AlgaeArmSettings.L2Feedforward.getKg());
            double upperJointKV = SmartDashboard.getNumber("Tune Algae upper joint kV", AlgaeArmSettings.L2Feedforward.getKv());
            if (!(lowerJointKS == AlgaeArmSettings.L1Feedforward.getKs() && lowerJointKG == AlgaeArmSettings.L1Feedforward.getKg() && lowerJointKV == AlgaeArmSettings.L1Feedforward.getKv())) {
                AlgaeArmSettings.L1Feedforward = new ArmFeedforward(lowerJointKS, lowerJointKG, lowerJointKV);
            }
            if (!(upperJointKS == AlgaeArmSettings.L2Feedforward.getKs() && upperJointKG == AlgaeArmSettings.L2Feedforward.getKg() && upperJointKV == AlgaeArmSettings.L2Feedforward.getKv())) {
                AlgaeArmSettings.L2Feedforward = new ArmFeedforward(upperJointKS, upperJointKG, upperJointKV);
            }
            
            //AlgaeArmSettings.maxAngleLowerJoint = SmartDashboard.getNumber("Tune Algae Limit maxAngleLowerJoint", AlgaeArmSettings.maxAngleLowerJoint);
            //AlgaeArmSettings.minAngleLowerJoint = SmartDashboard.getNumber("Tune Algae Limit minAngleLowerJoint", AlgaeArmSettings.minAngleLowerJoint);
            //AlgaeArmSettings.maxAngleUpperJointFromLower = SmartDashboard.getNumber("Tune Algae Limit maxAngleUpperJointFromLower", AlgaeArmSettings.maxAngleUpperJointFromLower);
            //AlgaeArmSettings.minAngleUpperJointFromLower = SmartDashboard.getNumber("Tune Algae Limit minAngleUpperJointFromLower", AlgaeArmSettings.minAngleUpperJointFromLower);
            //AlgaeArmSettings.maxDistanceInX = SmartDashboard.getNumber("Tune Algae Limit maxDistanceInX", AlgaeArmSettings.maxDistanceInX);
            //AlgaeArmSettings.maxDistanceOutX = SmartDashboard.getNumber("Tune Algae Limit maxDistanceOutX", AlgaeArmSettings.maxDistanceOutX);
            //AlgaeArmSettings.maxDistanceDownY = SmartDashboard.getNumber("Tune Algae Limit maxDistanceDownY", AlgaeArmSettings.maxDistanceDownY);

            AlgaeArmSettings.maxJoystickMovementSpeed = SmartDashboard.getNumber("Tune Algae Manual Control Speed", AlgaeArmSettings.maxJoystickMovementSpeed);
        }
        

        // Update encoders
        CurrentL1Angle = L1Encoder.getAsDouble();
        CurrentL2Angle = L2Encoder.getAsDouble();

        // Make sure final target is reachable
        if (Target.mag() > AlgaeArmConstants.LowerSegmentLength + AlgaeArmConstants.UpperSegmentLength) {
            Target.setMag(AlgaeArmConstants.LowerSegmentLength + AlgaeArmConstants.UpperSegmentLength);
        } else if (Target.mag() < Math.abs(AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength)) {
            Target.setMag(Math.abs(AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength));
        }

        Target.x = Functions.minMaxValue(AlgaeArmSettings.maxDistanceInX, AlgaeArmSettings.maxDistanceOutX, Target.x);
        if (Target.y < AlgaeArmSettings.maxDistanceDownY) Target.y = AlgaeArmSettings.maxDistanceDownY;

        /*
        double TargetL1Angle = getTargetLowerAngle();
        double TargetL2Angle = getTargetUpperAngle();
        TargetL1Angle = Functions.minMaxValue(AlgaeArmSettings.minAngleLowerJoint, AlgaeArmSettings.maxAngleLowerJoint, TargetL1Angle);
        TargetL2Angle = Functions.minMaxValue(AlgaeArmSettings.minAngleUpperJointFromLower + TargetL1Angle, AlgaeArmSettings.maxAngleUpperJointFromLower + TargetL1Angle, TargetL2Angle);
        setTargetAngles(TargetL1Angle, TargetL2Angle);
        */

        // Get moving target from motion profile
        motionProfile.setFinalTarget(Target);
        MovingTarget = motionProfile.update();
        MovingTarget = Target;

        // Make sure motion profile target is within reachable positions and won't clip into things
        if (MovingTarget.mag() > AlgaeArmConstants.LowerSegmentLength + AlgaeArmConstants.UpperSegmentLength) {
            MovingTarget.setMag(AlgaeArmConstants.LowerSegmentLength + AlgaeArmConstants.UpperSegmentLength);
        } else if (MovingTarget.mag() < Math.abs(AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength)) {
            MovingTarget.setMag(Math.abs(AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength));
        }

        MovingTarget.x = Functions.minMaxValue(AlgaeArmSettings.maxDistanceInX, AlgaeArmSettings.maxDistanceOutX, MovingTarget.x);
        if (MovingTarget.y < AlgaeArmSettings.maxDistanceDownY) MovingTarget.y = AlgaeArmSettings.maxDistanceDownY;

        // Use inverse kinematics to get new target joint angles and pass that to the PIDs
        double L1MotorPower = AlgaeArmSettings.LowerJointPID.calculate(CurrentL1Angle, getTargetLowerAngle());
        double L2MotorPower = AlgaeArmSettings.UpperJointPID.calculate(CurrentL2Angle, getTargetUpperAngle());

        // Add the gravity/acceleration compensation power
        if (AlgaeArmSettings.useFeedforward) {
            L1MotorPower += AlgaeArmSettings.L1Feedforward.calculate(getMovingTargetLowerAngle(), getLowerJointTargetAngVel());
            L2MotorPower += AlgaeArmSettings.L2Feedforward.calculate(getMovingTargetUpperAngle(), getUpperJointTargetAngVel());
        }
        
        // Give power to motors
        if (!armStopped) {
            bottomPivot.setVoltage(toVoltage(L1MotorPower + Math.cos(CurrentL1Angle)*AlgaeArmSettings.temporaryStaticPower));
            topPivot.setVoltage(toVoltage(L2MotorPower));
            clawRoller.set(clawRollerPower);
        } else {
            bottomPivot.setVoltage(0);
            topPivot.setVoltage(0);
            clawRoller.set(0);
        }
        


        //benisbestcoder();
        //public aidansucksatcode(boolean input) { return false; }
        //parker_is_lowkey_useless();
        //mael_is_hot();
        //BRUCE BRUCE BRUCE BRUCE BRUCE BRUCE
        //I DON'T LIKE VINCENT();


        // Telemetry
        double UpdateHz = -1;
        if (frameTime > 0) UpdateHz = 1.0 / frameTime;
        SmartDashboard.putNumber("Algae Arm framerate (hz)", UpdateHz);
        SmartDashboard.putString("Algae Arm Target", "x:" + round(Target.x, 2) + " y:" + round(Target.y, 2));
        SmartDashboard.putString("Algae Arm Moving Target", "x:" + round(MovingTarget.x, 2) + " y:" + round(MovingTarget.y, 2));
        Vector2d CurrentPointForTelemetry = getCurrentPoint();
        SmartDashboard.putString("Algae Arm Current Position", "x:" + round(CurrentPointForTelemetry.x, 2) + " y:" + round(CurrentPointForTelemetry.y, 2));
        Vector2d PositionErrorForTelemetry = MovingTarget.minus(CurrentPointForTelemetry);
        SmartDashboard.putString("Algae Arm Target Error", "x:" + round(PositionErrorForTelemetry.x, 3) + " y:" + round(PositionErrorForTelemetry.y, 3));
        SmartDashboard.putNumber("Algae Lower Angle", Math.toDegrees(CurrentL1Angle));
        SmartDashboard.putNumber("Algae Upper Angle", Math.toDegrees(CurrentL2Angle));
        SmartDashboard.putNumber("Algae Lower Target Angle", Math.toDegrees(TargetL1Angle));
        SmartDashboard.putNumber("Algae Upper Target Angle", Math.toDegrees(TargetL2Angle));
        SmartDashboard.putBoolean("Algae RightBias", RightBias);
        SmartDashboard.putNumber("Algae Lower Joint Target AngVel", getLowerJointTargetAngVel());
        SmartDashboard.putNumber("Algae Upper Joint Target AngVel", getUpperJointTargetAngVel());
        SmartDashboard.putNumber("Algae Lower Motor Power", L1MotorPower);
        SmartDashboard.putNumber("Algae Upper Motor Power", L2MotorPower);
        SmartDashboard.putNumber("Algae Lower Motor Current", bottomPivot.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Algae Upper Motor Current", topPivot.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Algae Claw Roller Power", clawRollerPower);
    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }


    private double toVoltage(double power){
        if(Math.abs(power)<AlgaeArmSettings.voltageTolerance)
        return 0;
        
        // power = (AlgaeArmConstants.maxVoltage-AlgaeArmConstants.minVoltage)*power + Math.signum(power)*AlgaeArmConstants.minVoltage; //adds the min value + the range between the max and min voltage to get a number between the min and max proportional to the power
        return power;
    }



    // KINEMATIC AND INVERSE KINEMATIC METHODS - sorry for making 623 different methods
    
    /**
     * Gets the current height and distance forward of the end of the arm based from the lower joint.
     */
    public Vector2d getCurrentPoint() {
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(CurrentL1Angle) + AlgaeArmConstants.UpperSegmentLength * Math.cos(CurrentL2Angle),
            AlgaeArmConstants.LowerSegmentLength * Math.sin(CurrentL1Angle) + AlgaeArmConstants.UpperSegmentLength * Math.sin(CurrentL2Angle)
        );
    }
    /**
     * Gets the current height and distance forward of the end of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getCurrentPointFromRobot() {
        return getCurrentPoint().plus(new Vector2d(AlgaeArmConstants.LowerJointForward, AlgaeArmConstants.LowerJointHeight));
    }

    /**
     * Gets the final target height and distance forward of the end of the arm based from the lower joint.
     */
    public Vector2d getTarget() {
        return Target;
    }
    /**
     * Gets the current motion profile target height and distance forward of the end of the arm based from the lower joint.
     */
    public Vector2d getMovingTarget() {
        return MovingTarget;
    }
    /**
     * Gets the target height and distance forward of the end of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getTargetFromRobot() {
        return Target.plus(new Vector2d(AlgaeArmConstants.LowerJointForward, AlgaeArmConstants.LowerJointHeight));
    }
    /**
     * Gets the current motion profile target height and distance forward of the end of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getMovingTargetFromRobot() {
        return MovingTarget.plus(new Vector2d(AlgaeArmConstants.LowerJointForward, AlgaeArmConstants.LowerJointHeight));
    }

    /**
     * Gets the current height and distance forward of the second joint of the arm based from the lower joint.
     */
    public Vector2d getCurrentMiddlePoint() {
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(CurrentL1Angle),
            AlgaeArmConstants.LowerSegmentLength * Math.sin(CurrentL1Angle)
        );
    }
    /**
     * Gets the current height and distance forward of the second joint of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getCurrentMiddlePointFromRobot() {
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(CurrentL1Angle) + AlgaeArmConstants.LowerJointForward,
            AlgaeArmConstants.LowerSegmentLength * Math.sin(CurrentL1Angle) + AlgaeArmConstants.LowerJointHeight
        );
    }
    /**
     * Gets the target height and distance forward of the second joint of the arm based from the lower joint.
     */
    public Vector2d getTargetMiddlePoint() {
        double L1Angle = getTargetLowerAngle();
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(L1Angle),
            AlgaeArmConstants.LowerSegmentLength * Math.sin(L1Angle)
        );
    }
    /**
     * Gets the target height and distance forward of the second joint of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getTargetMiddlePointFromRobot() {
        double L1Angle = getTargetLowerAngle();
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(L1Angle) + AlgaeArmConstants.LowerJointForward,
            AlgaeArmConstants.LowerSegmentLength * Math.sin(L1Angle) + AlgaeArmConstants.LowerJointHeight
        );
    }
    /**
     * Gets the target height and distance forward of the second joint of the arm based from the lower joint.
     */
    public Vector2d getMovingTargetMiddlePoint() {
        double L1Angle = getMovingTargetLowerAngle();
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(L1Angle),
            AlgaeArmConstants.LowerSegmentLength * Math.sin(L1Angle)
        );
    }
    /**
     * Gets the target height and distance forward of the second joint of the arm based from the center of the robot frame and the floor
     */
    public Vector2d getMovingTargetMiddlePointFromRobot() {
        double L1Angle = getMovingTargetLowerAngle();
        return new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(L1Angle) + AlgaeArmConstants.LowerJointForward,
            AlgaeArmConstants.LowerSegmentLength * Math.sin(L1Angle) + AlgaeArmConstants.LowerJointHeight
        );
    }

    public double getCurrentLowerAngle() { return Functions.normalizeAngle(CurrentL1Angle); }

    public double getCurrentUpperAngle() { return Functions.normalizeAngle(CurrentL2Angle); }

    public double getTargetLowerAngle() { 
        double midJointAngle = getTargetMidJointAngle();
        if (AlgaeArmConstants.UpperSegmentLength * Math.sin(Math.acos(AlgaeArmConstants.LowerSegmentLength / AlgaeArmConstants.UpperSegmentLength)) <= Target.mag()) {
            return Functions.normalizeAngle(Target.angle() + Math.asin((AlgaeArmConstants.UpperSegmentLength * Math.sin(midJointAngle)) / Target.mag()));
        } else return Functions.normalizeAngle(Target.angle() + Math.PI - Math.asin((AlgaeArmConstants.UpperSegmentLength * Math.sin(midJointAngle)) / Target.mag()));
    }

    public double getMovingTargetLowerAngle() { 
        double midJointAngle = getMovingTargetMidJointAngle();
        if (AlgaeArmConstants.UpperSegmentLength * Math.sin(Math.acos(AlgaeArmConstants.LowerSegmentLength / AlgaeArmConstants.UpperSegmentLength)) <= MovingTarget.mag()) {
            return Functions.normalizeAngle(MovingTarget.angle() + Math.asin((AlgaeArmConstants.UpperSegmentLength * Math.sin(midJointAngle)) / MovingTarget.mag()));
        } else return Functions.normalizeAngle(MovingTarget.angle() + Math.PI - Math.asin((AlgaeArmConstants.UpperSegmentLength * Math.sin(midJointAngle)) / MovingTarget.mag()));
    }

    public double getTargetUpperAngle() {
        if (RightBias) return Functions.normalizeAngle(getTargetLowerAngle() - getTargetMidJointAngle() + Math.PI);
        else return Functions.normalizeAngle(getTargetLowerAngle() + getTargetMidJointAngle() - Math.PI);
    }

    public double getMovingTargetUpperAngle() {
        if (RightBias) return Functions.normalizeAngle(getMovingTargetLowerAngle() - getMovingTargetMidJointAngle() + Math.PI);
        else return Functions.normalizeAngle(getMovingTargetLowerAngle() + getMovingTargetMidJointAngle() - Math.PI);
    }

    /**
     * Finds the current angle between the two arm segments
     */
    public double getCurrentMidJointAngle() {
        Vector2d CurrentEndPoint = getCurrentPoint();
        double result = Math.acos(((CurrentEndPoint.x * CurrentEndPoint.x + CurrentEndPoint.y * CurrentEndPoint.y) - AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength * AlgaeArmConstants.UpperSegmentLength) 
            / (-2 * AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.UpperSegmentLength));
        if (RightBias) return Functions.normalizeAngle(-1 * result);
        else return Functions.normalizeAngle(result);
    }

    /**
     * Finds the angle between the two arm segments when it is at its final target
     */
    public double getTargetMidJointAngle() {
        double result = Math.acos(((Target.x * Target.x + Target.y * Target.y) - AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength * AlgaeArmConstants.UpperSegmentLength) 
            / (-2 * AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.UpperSegmentLength));
        if (RightBias) return Functions.normalizeAngle(-1 * result);
        else return Functions.normalizeAngle(result);
    }

    /**
     * Finds the angle between the two arm segments when it is at its moving target
     */
    public double getMovingTargetMidJointAngle() {
        double result = Math.acos(((MovingTarget.x * MovingTarget.x + MovingTarget.y * MovingTarget.y) - AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength * AlgaeArmConstants.UpperSegmentLength) 
            / (-2 * AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.UpperSegmentLength));
        if (RightBias) return Functions.normalizeAngle(-1 * result);
        else return Functions.normalizeAngle(result);
    }




    // SET TARGET METHODS
    
    /**
     * Sets the target height and distance forward of the end of the arm based from the lower joint.
     */
    public void setTarget(Vector2d target) {
        Target = target;
    }

    public void setTargetLowerAngle(double lowerTargetAngle) { setTargetAngles(lowerTargetAngle, getTargetUpperAngle()); }

    public void setTargetUpperAngle(double upperTargetAngle) { setTargetAngles(getTargetLowerAngle(), upperTargetAngle); }
    
    /**
     * Sets the target height and distance forward of the end of the arm based from the center of the robot frame and the floor
     */
    public void setTargetPointFromRobot(Vector2d target) { Target = target.minus(new Vector2d(AlgaeArmConstants.LowerJointForward, AlgaeArmConstants.LowerJointHeight)); }

    public void setTargetAngles(double lowerTargetAngle, double upperTargetAngle) {
        lowerTargetAngle = Functions.minMaxValue(AlgaeArmSettings.minAngleLowerJoint, AlgaeArmSettings.maxAngleLowerJoint, lowerTargetAngle);
        upperTargetAngle = Functions.minMaxValue(AlgaeArmSettings.minAngleUpperJointFromLower + lowerTargetAngle, AlgaeArmSettings.maxAngleUpperJointFromLower + lowerTargetAngle, upperTargetAngle);
        Target = new Vector2d(
            AlgaeArmConstants.LowerSegmentLength * Math.cos(lowerTargetAngle) + AlgaeArmConstants.UpperSegmentLength * Math.cos(upperTargetAngle),
            AlgaeArmConstants.LowerSegmentLength * Math.sin(lowerTargetAngle) + AlgaeArmConstants.UpperSegmentLength * Math.sin(upperTargetAngle)
        );
    }





    // OTHER METHODS

    /**
     * Moves the target by the velocity inputted
     * @param x - in/s
     * @param y - in/s
     */
    public void setTargetMoveSpeeds(double x, double y) {
        if (Math.abs(x) < 0.05) x = 0;
        if (Math.abs(y) < 0.05) y = 0;
        Target = new Vector2d(Target.x + x * frameTime, Target.y + y * frameTime);
    }

    public void stopArm() {
        Target = getCurrentPoint();
        armStopped = true;
    }

    public void enableArm() {
        armStopped = false;
    }

    public boolean isCloseToTarget() {
        return Target.distFrom(getCurrentPoint()) < AlgaeArmSettings.PositionTolerance;
    }

    public boolean isCloseToTarget(Vector2d target) {
        return target.distFrom(getCurrentPoint()) < AlgaeArmSettings.PositionTolerance;
    }

    public void setRightBias(boolean rightBias) { RightBias = rightBias; }

    public boolean getRightBias() { return RightBias; }

    public void setTargetMaxInDirection(double angle) {
        Target.setMag(AlgaeArmConstants.LowerSegmentLength + AlgaeArmConstants.UpperSegmentLength);
        Target.setAngle(angle);
    }


    // These methods were utter torture and pain and suffering // TODO: Fix these with new functions
    public double getLowerJointTargetAngVel() { 
        Vector2d firstPoint = motionProfile.getCurrentTarget();
        Vector2d secondPoint = firstPoint.plus(motionProfile.getTargetVelocity());

        double UpperFirstAngle = 0;
        double UpperSecondAngle = 0;
        double result = Math.acos(((firstPoint.x * firstPoint.x + firstPoint.y * firstPoint.y) - AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength * AlgaeArmConstants.UpperSegmentLength) 
            / (-2 * AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.UpperSegmentLength));
        if (RightBias) UpperFirstAngle = Functions.normalizeAngle(-1 * result);
        else UpperFirstAngle = Functions.normalizeAngle(result);
        double result2 = Math.acos(((secondPoint.x * secondPoint.x + secondPoint.y * secondPoint.y) - AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength * AlgaeArmConstants.UpperSegmentLength) 
            / (-2 * AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.UpperSegmentLength));
        if (RightBias) UpperSecondAngle = Functions.normalizeAngle(-1 * result2);
        else UpperSecondAngle = Functions.normalizeAngle(result2);

        double firstAngle = Functions.normalizeAngle(firstPoint.angle() + Math.asin((AlgaeArmConstants.UpperSegmentLength * Math.sin(UpperFirstAngle)) / firstPoint.mag()));
        double secondAngle = Functions.normalizeAngle(secondPoint.angle() + Math.asin((AlgaeArmConstants.UpperSegmentLength * Math.sin(UpperSecondAngle)) / secondPoint.mag()));

        return secondAngle - firstAngle;
    }
    public double getUpperJointTargetAngVel() { // this is stupid
        Vector2d firstPoint = motionProfile.getCurrentTarget(); // get motion profile target
        Vector2d secondPoint = firstPoint.plus(motionProfile.getTargetVelocity()); // get a second point from adding the first point to the motion profile velocity

        // calculate the needed angle between each arm segment for each of the points
        double firstMidAngle = 0; 
        double secondMidAngle = 0;
        double result = Math.acos(((firstPoint.x * firstPoint.x + firstPoint.y * firstPoint.y) - AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength * AlgaeArmConstants.UpperSegmentLength) 
            / (-2 * AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.UpperSegmentLength));
        if (RightBias) firstMidAngle = Functions.normalizeAngle(-1 * result);
        else firstMidAngle = Functions.normalizeAngle(result);
        double result2 = Math.acos(((secondPoint.x * secondPoint.x + secondPoint.y * secondPoint.y) - AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.LowerSegmentLength - AlgaeArmConstants.UpperSegmentLength * AlgaeArmConstants.UpperSegmentLength) 
            / (-2 * AlgaeArmConstants.LowerSegmentLength * AlgaeArmConstants.UpperSegmentLength));
        if (RightBias) secondMidAngle = Functions.normalizeAngle(-1 * result2);
        else secondMidAngle = Functions.normalizeAngle(result2);

        // calculate the needed lower joint angle for each of the points
        double firstLowerAngle = 0;
        double secondLowerAngle = 0;
        if (AlgaeArmConstants.UpperSegmentLength * Math.sin(Math.acos(AlgaeArmConstants.LowerSegmentLength / AlgaeArmConstants.UpperSegmentLength)) <= firstPoint.mag()) {
            firstLowerAngle = Functions.normalizeAngle(firstPoint.angle() + Math.asin((AlgaeArmConstants.UpperSegmentLength * Math.sin(firstMidAngle)) / firstPoint.mag()));
        } else firstLowerAngle = Functions.normalizeAngle(firstPoint.angle() + Math.PI - Math.asin((AlgaeArmConstants.UpperSegmentLength * Math.sin(firstMidAngle)) / firstPoint.mag()));
        if (AlgaeArmConstants.UpperSegmentLength * Math.sin(Math.acos(AlgaeArmConstants.LowerSegmentLength / AlgaeArmConstants.UpperSegmentLength)) <= secondPoint.mag()) {
            secondLowerAngle = Functions.normalizeAngle(secondPoint.angle() + Math.asin((AlgaeArmConstants.UpperSegmentLength * Math.sin(secondMidAngle)) / secondPoint.mag()));
        } else secondLowerAngle = Functions.normalizeAngle(secondPoint.angle() + Math.PI - Math.asin((AlgaeArmConstants.UpperSegmentLength * Math.sin(secondMidAngle)) / secondPoint.mag()));

        // then by using both the needed angle between each arm segment and the needed lower joint angle, calculate the needed angle of the second joint for each of the points
        double firstAngle = 0;
        double secondAngle = 0;
        if (RightBias) firstAngle = Functions.normalizeAngle(firstLowerAngle - firstMidAngle + Math.PI);
        else firstAngle = Functions.normalizeAngle(firstLowerAngle + firstMidAngle - Math.PI);
        if (RightBias) secondAngle = Functions.normalizeAngle(secondLowerAngle - secondMidAngle + Math.PI);
        else secondAngle = Functions.normalizeAngle(secondLowerAngle + secondMidAngle - Math.PI);

        // and then finally find the difference between those angles from each point
        return secondAngle - firstAngle;
    }


    public void realignAlgaeArm() {
        L1Offset = AlgaeArmSettings.AlgaeArmLowerJointStartAngle - bottomPivot.getPosition().getValueAsDouble() * lowerGearRatio;
        L2Offset = AlgaeArmSettings.AlgaeArmUpperJointStartAngle - topPivot.getPosition().getValueAsDouble() * upperGearRatio;
        // initialize encoder suppliers
        L1Encoder = () -> bottomPivot.getPosition().getValueAsDouble() * lowerGearRatio + L1Offset; //I think this should get the currect position but will need testing
        L2Encoder = () -> topPivot.getPosition().getValueAsDouble() * upperGearRatio + L2Offset - L1Encoder.getAsDouble() + AlgaeArmSettings.AlgaeArmLowerJointStartAngle; // this last part is because I didn't understand the mechanism exactly which really complicated kinematics 
    }


    public void runRollerClaw(double speed) { clawRollerPower = speed; }
    public void stopRollerClaw() { clawRollerPower = 0; }
    public void intakeRollerClaw() { clawRollerPower = AlgaeRollerSettings.IntakePower; }
    public void holdRollerClaw() { clawRollerPower = AlgaeRollerSettings.HoldPower; }
    public void outtakeRollerClaw() { clawRollerPower = AlgaeRollerSettings.OuttakePower; }

}
