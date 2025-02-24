// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Settings.AlgaeArmSettings;
import frc.robot.commands.AlgaeArmTeleOpCommand;
import frc.robot.commands.CoralArmTeleOpCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeArmSystem;
import frc.robot.subsystems.ClimbingSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArmSystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final AlgaeArmSystem algaeArmSystem = new AlgaeArmSystem(AlgaeArmSettings.AlgaeArmLowerJointStartAngle, AlgaeArmSettings.AlgaeArmUpperJointStartAngle);
    public final ClimbingSystem climbingSystem = new ClimbingSystem(operatorJoystick.rightBumper(), operatorJoystick.rightTrigger());
    public final CoralArmSystem coralArmSystem = new CoralArmSystem();

    private final CoralArmTeleOpCommand runElevatorUp = new CoralArmTeleOpCommand(coralArmSystem, 1);
    private final CoralArmTeleOpCommand runElevatorDown = new CoralArmTeleOpCommand(coralArmSystem, -1);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        operatorJoystick.leftBumper().onTrue(runElevatorUp);
        operatorJoystick.leftTrigger().onTrue(runElevatorDown);

        operatorJoystick.b().onTrue(new InstantCommand(algaeArmSystem::realignAlgaeArm));

        // temporary Algae Arm controls
        algaeArmSystem.setDefaultCommand(new AlgaeArmTeleOpCommand(algaeArmSystem, 
            () -> -operatorJoystick.getLeftY() * AlgaeArmSettings.maxJoystickMovementSpeed, 
            () -> -operatorJoystick.getLeftX() * AlgaeArmSettings.maxJoystickMovementSpeed
        ));
        //Coral Elevator Controls
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
