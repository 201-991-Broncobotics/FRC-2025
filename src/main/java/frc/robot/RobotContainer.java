// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
//import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Settings.AlgaeArmSettings;
import frc.robot.commands.AlgaeArmTeleOpCommand;
import frc.robot.commands.Autonomous;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeArmSystem;
import frc.robot.subsystems.ClimbingSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralElevatorSystem;
import frc.robot.subsystems.FalconCoralClaw;
import frc.robot.subsystems.DrivingProfiles;
import frc.robot.subsystems.Vision;
import frc.robot.utility.Functions;

public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband - now 6%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);
    private final Joystick driverFlightHotasOne = new Joystick(2);

    private final Vision vision = new Vision();

    private final DrivingProfiles drivingProfile = new DrivingProfiles(vision);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final AlgaeArmSystem algaeArmSystem = new AlgaeArmSystem(AlgaeArmSettings.AlgaeArmLowerJointStartAngle, AlgaeArmSettings.AlgaeArmUpperJointStartAngle);
    public final ClimbingSystem climbingSystem = new ClimbingSystem();

   public final CoralElevatorSystem coralElevatorSystem = new CoralElevatorSystem();


    //public final CoralArmSystem coralArmSystem = new CoralArmSystem("test up");
    //public final FalconCoralClaw coralClawSystem = new FalconCoralClaw();

    //private final CoralArmTeleOpCommand runElevatorUp = new CoralArmTeleOpCommand(coralArmSystem, 1);
    //private final CoralArmTeleOpCommand runElevatorDown = new CoralArmTeleOpCommand(coralArmSystem, -1);
    //DoubleSupplier sup = () -> operatorJoystick.getRightY();
    //public final CoralArmSystem coralArmSystem = new CoralArmSystem(sup);

    //pathplanner auto
    private final SendableChooser<Command> autoChooser; 

    public RobotContainer() {
         //Register Auto Commands
        NamedCommands.registerCommand("startArm",new InstantCommand(algaeArmSystem::enableArm));
        NamedCommands.registerCommand("raiseArm",new InstantCommand(algaeArmSystem::presetAutoScore));
        NamedCommands.registerCommand("placeCoral",new InstantCommand(algaeArmSystem::outtakeRollerClaw));
        NamedCommands.registerCommand("stopClaw",new InstantCommand(algaeArmSystem::stopRollerClaw));
        
        drivetrain.configureAutoBuilder();
        //selecting the pathplanner auto you want from dashboard + setting default
        autoChooser = AutoBuilder.buildAutoChooser("middleAuto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivingProfile.setUpControllerInputs(
            () -> -driverJoystick.getLeftY(), 
            () -> driverJoystick.getLeftX(), 
            () -> -driverJoystick.getRightX(), 
            () -> 0.5 + 0.5 * driverJoystick.getRightTriggerAxis(), 
            2, 2
        );

        drivingProfile.setUpJoystickInputs(
            () -> driverFlightHotasOne.getY(), 
            () -> -driverFlightHotasOne.getX(), 
            () -> -driverFlightHotasOne.getRawAxis(5), 
            () -> 0.2 + 0.8 * ((-driverFlightHotasOne.getRawAxis(2)+1)/2), 
            1, 2
        );  

        // drivingProfile.giveJoystickForTelemetry(driverFlightHotasOne);

        drivingProfile.setDefaultCommand(new RunCommand(drivingProfile::update, drivingProfile));

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(drivingProfile.getForwardOutput() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-drivingProfile.getStrafeOutput() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(drivingProfile.getRotationOutput() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )

            // old
            /* 
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed * (0.5 + 0.5 * Math.abs(driverJoystick.getRightTriggerAxis()))) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed * (0.5 + 0.5 * Math.abs(driverJoystick.getRightTriggerAxis()))) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate * (0.6 + 0.4 * Math.abs(driverJoystick.getRightTriggerAxis()))) // Drive counterclockwise with negative X (left)
            )
            */
        );

            
        // DRIVER CONTROLS

        // Auto Targeting
        drivingProfile.setUpAutoThrottleControllerInput(() -> driverJoystick.getLeftTriggerAxis());
        drivingProfile.setUpAutoThrottleJoystickInput(() -> 0.2 + 0.8 * ((-driverFlightHotasOne.getRawAxis(2)+1)/2));

        //driverJoystick.leftBumper().onTrue(new InstantCommand(drivingProfile::enableAutoAim)).onFalse(new InstantCommand(drivingProfile::disableAutoAim));
        driverJoystick.leftTrigger().onTrue(new InstantCommand(drivingProfile::enableAutoStrafing)).onFalse(new InstantCommand(drivingProfile::disableAutoStrafing));
        new JoystickButton(driverFlightHotasOne, 1).onTrue(new InstantCommand(drivingProfile::enableAutoAim)).onFalse(new InstantCommand(drivingProfile::disableAutoAim));
        new JoystickButton(driverFlightHotasOne, 15).onTrue(new InstantCommand(drivingProfile::enableAutoDriving)).onFalse(new InstantCommand(drivingProfile::disableAutoDriving));



        driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> brake));
        new JoystickButton(driverFlightHotasOne, 7).whileTrue(drivetrain.applyRequest(() -> brake)); // d button on throttle side

        // controls for checking if the wheels are aligned
        driverJoystick.a().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))
        ));
        new JoystickButton(driverFlightHotasOne, 15).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-1, 0)) // mouseclick button on throttle side
        ));

        new JoystickButton(driverFlightHotasOne, 6).onTrue(new InstantCommand(drivingProfile::enableSlowDown)).onFalse(new InstantCommand(drivingProfile::disableSlowDown));

        // reset the field-centric heading on left bumper press
        driverJoystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        new JoystickButton(driverFlightHotasOne, 5).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Climbing
        //driverJoystick.povUp().onTrue(new InstantCommand(climbingSystem::StartUnclimbing)).toggleOnFalse(new InstantCommand(climbingSystem::StopClimbing));
        //driverJoystick.povDown().onTrue(new InstantCommand(climbingSystem::StartClimbing)).toggleOnFalse(new InstantCommand(climbingSystem::StopClimbing));

        new JoystickButton(driverFlightHotasOne, 14).onTrue(new InstantCommand(climbingSystem::StartClimbing)).onFalse(new InstantCommand(climbingSystem::StopClimbing));
        new JoystickButton(driverFlightHotasOne, 13).onTrue(new InstantCommand(climbingSystem::StartUnclimbing)).onFalse(new InstantCommand(climbingSystem::StopClimbing));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        




        // OPERATOR CONTROLS

        coralElevatorSystem.setManualControl(() -> ((operatorJoystick.leftBumper().getAsBoolean())? 0.5:0.0) + ((operatorJoystick.getLeftTriggerAxis() > 0.25)? -0.5:0.0)); //-Functions.deadbandValue(operatorJoystick.getLeftY(),  0.1));
        coralElevatorSystem.setManualPivotControl(() -> -Functions.deadbandValue(operatorJoystick.getLeftY(),  0.05));

        //operatorJoystick.leftBumper().onTrue(runElevatorUp);
        //operatorJoystick.leftTrigger().onTrue(runElevatorDown);
        //operatorJoystick.rightBumper().onTrue(new InstantCommand(coralClawSystem::outtakeRoller)).onFalse(new InstantCommand(coralClawSystem::stopRoller));
        //operatorJoystick.rightTrigger().onTrue(new InstantCommand(coralClawSystem::intakeRoller)).toggleOnFalse(new InstantCommand(coralClawSystem::holdRoller));
        operatorJoystick.rightBumper().onTrue(new InstantCommand(algaeArmSystem::outtakeRollerClaw)).onFalse(new InstantCommand(algaeArmSystem::stopRollerClaw));
        operatorJoystick.leftBumper().onTrue(new InstantCommand(algaeArmSystem::shootRollerClaw)).onFalse(new InstantCommand(algaeArmSystem::stopRollerClaw));
        operatorJoystick.rightTrigger().onTrue(new InstantCommand(algaeArmSystem::intakeRollerClaw)).toggleOnFalse(new InstantCommand(algaeArmSystem::holdRollerClaw));

        
        //operatorJoystick.povDown().onTrue(new InstantCommand(algaeArmSystem::toggleRightBias));
        //operatorJoystick.povUp().onTrue(new InstantCommand(coralArmSystem::toggleCoralArm));

        // operatorJoystick.b().onTrue(new InstantCommand(algaeArmSystem::realignAlgaeArm));
        operatorJoystick.y().toggleOnTrue(new InstantCommand(coralElevatorSystem::ToggleEnabled));
        operatorJoystick.x().toggleOnTrue(new InstantCommand(coralElevatorSystem::ArmToggleEnabled));

        operatorJoystick.povDown().onTrue(new InstantCommand(algaeArmSystem::presetFloorForward));
        //operatorJoystick.povUp().onTrue(new InstantCommand(algaeArmSystem::presetHighBall));
        operatorJoystick.povLeft().onTrue(new InstantCommand(algaeArmSystem::presetLowBall));
        //operatorJoystick.povRight().onTrue(new InstantCommand(algaeArmSystem::presetFloorBackward));

        operatorJoystick.leftBumper().onTrue(new InstantCommand(algaeArmSystem::disableLimits)).onFalse(new InstantCommand(algaeArmSystem::enableLimits));

        //operatorJoystick.povDownLeft().onTrue(new InstantCommand(coralClawSystem::toggleLeftDiffyPosition));
        //operatorJoystick.povDownRight().onTrue(new InstantCommand(coralArmSystem::toggleStoreArm));

        // temporary Algae Arm controls
        algaeArmSystem.setControllerInputsJoint( 
            () -> operatorJoystick.getLeftY(), 
            () -> operatorJoystick.getRightY(),
            () -> 1.0 - 0.5 * operatorJoystick.getLeftTriggerAxis()
        );

        //coralClawSystem.setDefaultCommand(new RunCommand(coralClawSystem::update, coralClawSystem));
        //coralArmSystem.setDefaultCommand(new RunCommand(coralArmSystem::update, coralArmSystem));
        algaeArmSystem.setDefaultCommand(new RunCommand(algaeArmSystem::updateInTeleOp, algaeArmSystem));
        climbingSystem.setDefaultCommand(new RunCommand(climbingSystem::update, climbingSystem));
        
        
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
