package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.robot.Constants.AutoDrivingConstants;
import frc.robot.Settings.AutoDrivingSettings;
import frc.robot.Settings.AutoTargetingSettings;
import frc.robot.subsystems.CommandSwerveDrivetrain.gyroData;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;
import frc.robot.utility.AdvancedPIDController;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.Functions;
import frc.robot.utility.Vector2d;

/**
 * This is to normalize driving and make it easier for Mael to not drive into people. 
 * It also determines which driver controller/joystick should take priority and allows them to be used interchangeably
 */
public class DrivingProfiles extends SubsystemBase {

    private DoubleSupplier fowardControllerInput, strafeControllerInput, rotationControllerInput, throttleControllerInput;
    private DoubleSupplier fowardJoystickInput, strafeJoystickInput, rotationJoystickInput, throttleJoystickInput;
    private DoubleSupplier autoThrottleControllerInput, autoThrottleJoystickInput;

    private double forwardOutput = 0, strafeOutput = 0, rotationOutput = 0;

    private double controllerDriveCurveMag, controllerTurnCurveMag;
    private double joystickDriveCurveMag, joystickTurnCurveMag;

    private boolean preferController = true;

    private final double ControllerDeadband = 0.05, JoystickDeadband = 0.13, AutoThrottleDeadband = 0.05;

    private double presetThrottleControl = 0.25;
    private boolean useThrottlePreset = false, autoAiming = false, autoDriving = false, autoStrafing = false;

    private double autoForwardOutput = 0, autoStrafeOutput = 0, autoRotationOutput = 0;

    private static CommandSwerveDrivetrain drivetrain;

    private boolean useAutoDrivingThrottle = true;
    private DoubleSupplier AutoDrivingThrottle;

    private static Pose2d RobotPose;

    private ElapsedTime FPSTimer;

    public List<List<Pose2d>> FieldTargetPoints; // Blue then red

    private Pose2d ClosestFieldTargetPoint = new Pose2d();


    public DrivingProfiles(CommandSwerveDrivetrain drivetrain, boolean PreferController) {
        DrivingProfiles.drivetrain = drivetrain;
        this.preferController = PreferController;

        RobotPose = drivetrain.getState().Pose;

        FPSTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        if (Settings.tuningTelemetryEnabled) {
            SmartDashboard.putNumber("Tune Reef Outward Offset", AutoDrivingSettings.OutwardFromCenter);
            SmartDashboard.putNumber("Tune Reef Right Offset", AutoDrivingSettings.RightFromCenter);
            SmartDashboard.putNumber("Tune Reef Left Offset", AutoDrivingSettings.LeftFromCenter);

            SmartDashboard.putNumber("Tune Auto Turning kP", AutoTargetingSettings.AutoTurningPID.getP());
            SmartDashboard.putNumber("Tune Auto Turning kI", AutoTargetingSettings.AutoTurningPID.getI());
            SmartDashboard.putNumber("Tune Auto Turning kD", AutoTargetingSettings.AutoTurningPID.getD());
            SmartDashboard.putNumber("Tune Auto Driving kP", AutoTargetingSettings.AutoDrivingPID.getP());
            SmartDashboard.putNumber("Tune Auto Driving kD", AutoTargetingSettings.AutoDrivingPID.getD());
            SmartDashboard.putNumber("Tune Auto Driving Max Power", AutoTargetingSettings.AutoDrivingMaxPower);

            SmartDashboard.putBoolean("Auto Aiming Enabled", AutoTargetingSettings.AutoAimingEnabled);
            SmartDashboard.putBoolean("Auto Driving Enabled", AutoTargetingSettings.AutoDrivingEnabled);
            SmartDashboard.putNumber("Auto Driving Power", AutoTargetingSettings.AutoDrivingPower);
            SmartDashboard.putNumber("Auto target percentage of blocked vision", AutoTargetingSettings.targetPercentageOfVisionBlocked);
        }

    }

    public DrivingProfiles(CommandSwerveDrivetrain drivetrain) {
        new DrivingProfiles(drivetrain, preferController);
    }


    public void setUpControllerInputs(DoubleSupplier fowardInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, DoubleSupplier throttleInput, double driveCurveMag, double turnCurveMag) {
        this.fowardControllerInput = fowardInput;
        this.strafeControllerInput = strafeInput;
        this.rotationControllerInput = rotationInput;
        this.throttleControllerInput = throttleInput;
        this.controllerDriveCurveMag = driveCurveMag;
        this.controllerTurnCurveMag = turnCurveMag;
    }

    public void setUpJoystickInputs(DoubleSupplier fowardInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, DoubleSupplier throttleInput, double driveCurveMag, double turnCurveMag) {
        this.fowardJoystickInput = fowardInput;
        this.strafeJoystickInput = strafeInput;
        this.rotationJoystickInput = rotationInput;
        this.throttleJoystickInput = throttleInput;
        this.joystickDriveCurveMag = driveCurveMag;
        this.joystickTurnCurveMag = turnCurveMag;
    }

    public void setUpAutoThrottleControllerInput(DoubleSupplier autoThrottleControllerInput) {
        this.autoThrottleControllerInput = autoThrottleControllerInput;
    }

    public void setUpAutoThrottleJoystickInput(DoubleSupplier autoThrottleJoystickInput) {
        this.autoThrottleJoystickInput = autoThrottleJoystickInput;
    }


    public void update() {
        if (preferController) {
            if (updateController());
            else if (updateJoystick());
            else stopDriving();
        } else {
            if (updateJoystick());
            else if (updateController());
            else stopDriving();
        }

        if (useAutoDrivingThrottle) autoDriving = (AutoDrivingThrottle.getAsDouble() > AutoThrottleDeadband);

        //if (autoAiming) updateAutoAiming();
        if (autoDriving) updateAutoDriving();
        //if (autoStrafing) updateAutoStrafing();
    }


    private boolean updateController() {
        double forward = fowardControllerInput.getAsDouble();
        double strafe = strafeControllerInput.getAsDouble();
        double turn = rotationControllerInput.getAsDouble();
        double throttle = throttleControllerInput.getAsDouble();

        if (useThrottlePreset) throttle = presetThrottleControl;

        double Direction = Math.atan2(forward, strafe);
        double joystickPower = Math.hypot(forward, strafe);
        double drivePower = Functions.throttleCurve(joystickPower, controllerDriveCurveMag) * throttle;

        if (joystickPower == 0.0) drivePower = 0; // just to make sure

        forwardOutput = Math.sin(Direction) * drivePower;
        strafeOutput = Math.cos(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, controllerTurnCurveMag) * throttle;

        return !(joystickPower == 0.0 && turn == 0.0); // returns true if in use
    }


    private boolean updateJoystick() {
        double forward = fowardJoystickInput.getAsDouble();
        double strafe = strafeJoystickInput.getAsDouble();
        double turn = Functions.deadbandValue(rotationJoystickInput.getAsDouble(), JoystickDeadband);
        double throttle = throttleJoystickInput.getAsDouble();

        if (useThrottlePreset) throttle = presetThrottleControl;

        double Direction = Math.atan2(forward, strafe);
        double joystickPower = Functions.deadbandValue(Math.hypot(forward, strafe), JoystickDeadband);
        double drivePower = Functions.throttleCurve(joystickPower, joystickDriveCurveMag) * throttle;

        if (joystickPower == 0.0) drivePower = 0; // just to make sure

        forwardOutput = Math.sin(Direction) * drivePower;
        strafeOutput = Math.cos(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, joystickTurnCurveMag) * throttle;

        return !(joystickPower == 0.0 && turn == 0.0); // returns true if in use
    }

    /* 
    private void updateAutoAiming() {
        autoRotationOutput = Functions.minMaxValue(-1, 1, AutoTargetingSettings.AutoTurningPID.calculate(LimelightHelpers.getTX("limelight"), 0.0));

        if (AutoTargetingSettings.AutoAimingEnabled) {
            if (LimelightHelpers.getTargetCount("limelight") > 0) {
                rotationOutput = autoRotationOutput;
            } else {
                if (lastSawObjectOnLeft) rotationOutput = AutoTargetingSettings.searchingSpeed;
                else rotationOutput = -AutoTargetingSettings.searchingSpeed;
            }
            
        }
    } */

    private void updateAutoDriving() {

        RobotPose = drivetrain.getState().Pose;
        if (RobotPose != null) {
            Vector2d autoDrivingDirection = new Vector2d()
                .withMag(Functions.minMaxValue(0, AutoTargetingSettings.AutoDrivingMaxPower, 
                    AutoTargetingSettings.AutoDrivingPID.calculate(0, 
                        (new Vector2d(RobotPose.getX(), RobotPose.getY())).distFrom(new Vector2d(ClosestFieldTargetPoint.getX(), ClosestFieldTargetPoint.getY()))
                    )))
                .withAngle(((new Vector2d(RobotPose.getX(), RobotPose.getY())).minus(new Vector2d(ClosestFieldTargetPoint.getX(), ClosestFieldTargetPoint.getY()))).angle() + Math.toRadians(90));

            double throttle = 0; /* 
            if (preferController) {
                if (autoThrottleControllerInput.getAsDouble() > AutoThrottleDeadband) throttle = autoThrottleControllerInput.getAsDouble();
                else throttle = autoThrottleJoystickInput.getAsDouble();
            } else {
                if (autoThrottleJoystickInput.getAsDouble() > AutoThrottleDeadband) throttle = autoThrottleJoystickInput.getAsDouble();
                else throttle = autoThrottleControllerInput.getAsDouble();
            } */

            throttle = Functions.deadbandValue(AutoDrivingThrottle.getAsDouble(), AutoThrottleDeadband);

            autoForwardOutput = autoDrivingDirection.y * throttle;
            autoStrafeOutput = autoDrivingDirection.x * throttle;
            autoRotationOutput = AutoTargetingSettings.AutoTurningPID.calculate(Functions.normalizeAngle(RobotPose.getRotation().getRadians() - ClosestFieldTargetPoint.getRotation().getRadians()), 0) * throttle;

            if (AutoTargetingSettings.AutoDrivingEnabled) {
                forwardOutput += autoForwardOutput;
                strafeOutput += autoStrafeOutput;
                rotationOutput += autoRotationOutput;
            }
            SmartDashboard.putString("AutoDriving", "is supposed to be working");
        } else SmartDashboard.putString("AutoDriving", "error");

        
    } 

    /* 

    private void updateAutoStrafing() {

        double cameraTX = LimelightHelpers.getTX("limelight");
        double distanceFromCenter = cameraTX;
        if (Math.abs(cameraTX - AutoTargetingSettings.leftCorrectX) <= Math.abs(cameraTX - AutoTargetingSettings.rightCorrectX)) {
            distanceFromCenter = cameraTX - AutoTargetingSettings.leftCorrectX;
        } else {
            distanceFromCenter = cameraTX - AutoTargetingSettings.rightCorrectX;
        }

        Vector2d autoStrafingDirection = new Vector2d()
            .withMag(AutoTargetingSettings.AutoDrivingPower * distanceFromCenter)
            .withAngle(RobotPose.getRotation().getRadians() + Math.toRadians(90));

        double throttle = 0;
        if (preferController) {
            if (autoThrottleControllerInput.getAsDouble() > AutoThrottleDeadband) throttle = autoThrottleControllerInput.getAsDouble();
            else throttle = autoThrottleJoystickInput.getAsDouble();
        } else {
            if (autoThrottleJoystickInput.getAsDouble() > AutoThrottleDeadband) throttle = autoThrottleJoystickInput.getAsDouble();
            else throttle = autoThrottleControllerInput.getAsDouble();
        }


        autoForwardOutput = autoStrafingDirection.y * throttle;
        autoStrafeOutput = autoStrafingDirection.x * throttle;

        if (AutoTargetingSettings.AutoDrivingEnabled && LimelightHelpers.getTargetCount("limelight") > 0) {
            forwardOutput += autoForwardOutput;
            strafeOutput += autoStrafeOutput;
        }

    } */


    public void stopDriving() {
        forwardOutput = 0;
        strafeOutput = 0;
        rotationOutput = 0;
    }


    public double getForwardOutput() { return forwardOutput; }
    public double getStrafeOutput() { return strafeOutput; }
    public double getRotationOutput() { return rotationOutput; }


    public void enableSlowDown() { useThrottlePreset = true; }
    public void disableSlowDown() { useThrottlePreset = false; }

    //public void enableAutoAim() { autoAiming = true; }
    //public void disableAutoAim() { autoAiming = false; }
    public void enableAutoDriving() { autoDriving = true; }
    public void disableAutoDriving() { autoDriving = false; }
    //public void enableAutoStrafing() { autoStrafing = true; }
    //public void disableAutoStrafing() { autoStrafing = false; }

    public void setupAutoDrivingThrottle(DoubleSupplier AutoThrottle) {
        useAutoDrivingThrottle = true;
        AutoDrivingThrottle = AutoThrottle;
    }


    private void CreateTargetFieldPoints() {
        FieldTargetPoints = new ArrayList<>();
        FieldTargetPoints.add(new ArrayList<>());
        FieldTargetPoints.add(new ArrayList<>());
        
        Vector2d RightSide = new Vector2d(AutoDrivingSettings.OutwardFromCenter, -AutoDrivingSettings.RightFromCenter); // create a vector from the center of the reef to each placing spot on one edge
        Vector2d LeftSide = new Vector2d(AutoDrivingSettings.OutwardFromCenter, AutoDrivingSettings.LeftFromCenter);

        for (int i = 0; i <= 5; i++) {
            double Angle = i*(Math.PI/3);
            Vector2d BlueRightSide = AutoDrivingConstants.BlueReefCenter.plus(RightSide.withAngle(Angle + RightSide.angle())); // then adds it to the location of each reef and rotates it 6 times for each edge of the reef
            Vector2d BlueLeftSide = AutoDrivingConstants.BlueReefCenter.plus(LeftSide.withAngle(Angle + LeftSide.angle()));
            Vector2d RedRightSide = AutoDrivingConstants.RedReefCenter.plus(RightSide.withAngle(Angle + RightSide.angle()));
            Vector2d RedLeftSide = AutoDrivingConstants.RedReefCenter.plus(LeftSide.withAngle(Angle + LeftSide.angle()));
            
            FieldTargetPoints.get(0).add(new Pose2d(BlueRightSide.x, BlueRightSide.y, new Rotation2d(Angle))); // and then adds them to the list
            FieldTargetPoints.get(0).add(new Pose2d(BlueLeftSide.x, BlueLeftSide.y, new Rotation2d(Angle)));
            FieldTargetPoints.get(1).add(new Pose2d(RedRightSide.x, RedRightSide.y, new Rotation2d(Angle))); 
            FieldTargetPoints.get(1).add(new Pose2d(RedLeftSide.x, RedLeftSide.y, new Rotation2d(Angle)));
        }
        
    }

    private Pose2d getClosestPoint(Pose2d pose) {
        Vector2d CurrentVector = new Vector2d(pose.getX(), pose.getY());
        List<Pose2d> closerReef;
        if (CurrentVector.distFrom(AutoDrivingConstants.BlueReefCenter) < CurrentVector.distFrom(AutoDrivingConstants.RedReefCenter)) {
            closerReef = FieldTargetPoints.get(0);
        } else {
            closerReef = FieldTargetPoints.get(1);
        }

        Pose2d PointWithLowestDistance = closerReef.get(0);
        for (Pose2d i : closerReef) {
            if (CurrentVector.distFrom(new Vector2d(i.getX(), i.getY())) < CurrentVector.distFrom(new Vector2d(PointWithLowestDistance.getX(), PointWithLowestDistance.getY()))) {
                PointWithLowestDistance = i;
            }
        }
        return PointWithLowestDistance;
    }


    @Override
    public void periodic() {
        if (FPSTimer != null) {
            if (FPSTimer.time() > 0) SmartDashboard.putNumber("FPS:", Functions.round(1.0 / FPSTimer.time(), 2));
            FPSTimer.reset();
        }
        
        if (drivetrain != null) {
            PoseEstimate LimelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
            if (LimelightHelpers.validPoseEstimate(LimelightPoseEstimate)) drivetrain.addVisionMeasurement(LimelightPoseEstimate.pose, LimelightPoseEstimate.timestampSeconds);

            RobotPose = drivetrain.getState().Pose;
            SmartDashboard.putString("ROBOT POSE:", "X:" + RobotPose.getX() + " Y:" + RobotPose.getY() + " R:" + RobotPose.getRotation().getDegrees());

            double cameraTX = LimelightHelpers.getTX("limelight");

            CreateTargetFieldPoints(); // refreshes the list of points each frame
            ClosestFieldTargetPoint = getClosestPoint(RobotPose);
            Vector2d CurrentVector = new Vector2d(RobotPose.getX(), RobotPose.getY());
            if (CurrentVector.distFrom(AutoDrivingConstants.BlueReefCenter) < CurrentVector.distFrom(AutoDrivingConstants.RedReefCenter)) {
                SmartDashboard.putString("Vision Closer Reef", "Blue");
            } else SmartDashboard.putString("Vision Closer Reef", "Red");
            SmartDashboard.putString("Vision TARGET:", "X:" + ClosestFieldTargetPoint.getX() + " Y:" + ClosestFieldTargetPoint.getY() + " R:" + ClosestFieldTargetPoint.getRotation().getDegrees());


            SmartDashboard.putNumber("Vision TX", cameraTX);
            SmartDashboard.putNumber("Vision TA", LimelightHelpers.getTA("limelight"));
            SmartDashboard.putBoolean("Vision valid Target", LimelightHelpers.getTargetCount("limelight") > 0);
            SmartDashboard.putBoolean("IS AUTO DRIVING?", autoDriving);
            SmartDashboard.putNumber("AUTO Driving forward", autoForwardOutput);
            SmartDashboard.putNumber("AUTO Driving strafe", autoStrafeOutput);
            SmartDashboard.putNumber("AUTO Driving rotation", autoRotationOutput);
        }
        
        
        
       
        //if (cameraTX > 0) lastSawObjectOnLeft = false;
        //else if (cameraTX < 0) lastSawObjectOnLeft = true;



        

        // SmartDashboard.putBoolean("Vision last saw object on left", lastSawObjectOnLeft);

        //update settings
        if (Settings.tuningTelemetryEnabled) {
            AutoDrivingSettings.OutwardFromCenter = SmartDashboard.getNumber("Tune Reef Outward Offset", AutoDrivingSettings.OutwardFromCenter);
            AutoDrivingSettings.RightFromCenter = SmartDashboard.getNumber("Tune Reef Right Offset", AutoDrivingSettings.RightFromCenter);
            AutoDrivingSettings.LeftFromCenter = SmartDashboard.getNumber("Tune Reef Left Offset", AutoDrivingSettings.LeftFromCenter);

            AutoTargetingSettings.AutoTurningPID.setP(SmartDashboard.getNumber("Tune Auto Turning kP", AutoTargetingSettings.AutoTurningPID.getP()));
            AutoTargetingSettings.AutoTurningPID.setI(SmartDashboard.getNumber("Tune Auto Turning kI", AutoTargetingSettings.AutoTurningPID.getI()));
            AutoTargetingSettings.AutoTurningPID.setD(SmartDashboard.getNumber("Tune Auto Turning kD", AutoTargetingSettings.AutoTurningPID.getD()));
            AutoTargetingSettings.AutoDrivingPID.setP(SmartDashboard.getNumber("Tune Auto Driving kP", AutoTargetingSettings.AutoDrivingPID.getP()));
            AutoTargetingSettings.AutoDrivingPID.setD(SmartDashboard.getNumber("Tune Auto Driving kD", AutoTargetingSettings.AutoDrivingPID.getD()));
            AutoTargetingSettings.AutoDrivingMaxPower = SmartDashboard.getNumber("Tune Auto Driving Max Power", AutoTargetingSettings.AutoDrivingMaxPower);

            AutoTargetingSettings.AutoAimingEnabled = SmartDashboard.getBoolean("Auto Aiming Enabled", AutoTargetingSettings.AutoAimingEnabled);
            AutoTargetingSettings.AutoDrivingEnabled = SmartDashboard.getBoolean("Auto Driving Enabled", AutoTargetingSettings.AutoDrivingEnabled);
            AutoTargetingSettings.AutoDrivingPower = SmartDashboard.getNumber("Auto Driving Power", AutoTargetingSettings.AutoDrivingPower);
            // AutoTargetingSettings.targetPercentageOfVisionBlocked = SmartDashboard.getNumber("Auto target percentage of blocked vision", AutoTargetingSettings.targetPercentageOfVisionBlocked);
        }

        /*
        SmartDashboard.putNumber("Pigeon accel X", gyroData.accelX);
        SmartDashboard.putNumber("Pigeon accel Y", gyroData.accelY);
        SmartDashboard.putNumber("Pigeon accel Z", gyroData.accelZ);
        SmartDashboard.putNumber("Pigeon pitch", gyroData.pitch);
        SmartDashboard.putNumber("Pigeon roll", gyroData.roll);
        
        SmartDashboard.putNumber("Pigeon angVel X", gyroData.angVelX);
        SmartDashboard.putNumber("Pigeon angVel Y", gyroData.angVelY);
        SmartDashboard.putNumber("Pigeon angVel Z", gyroData.angVelZ);
        */
        
    }

}
