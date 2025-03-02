// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
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
import frc.robot.commands.DrivingJoystickProfile;
import frc.robot.commands.DrivingProfile;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeArmSystem;
import frc.robot.subsystems.ClimbingSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralArmSystem;
import frc.robot.subsystems.CoralClaw;
import frc.robot.utility.Functions;

public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband - now 6%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);
    private final Joystick driverFlightHotasOne = new Joystick(2);

    private DrivingProfile drivingProfile;
    private DrivingJoystickProfile drivingJoystickProfile;
    

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final AlgaeArmSystem algaeArmSystem = new AlgaeArmSystem(AlgaeArmSettings.AlgaeArmLowerJointStartAngle, AlgaeArmSettings.AlgaeArmUpperJointStartAngle);
    public final ClimbingSystem climbingSystem = new ClimbingSystem();
    //public final CoralArmSystem coralArmSystem = new CoralArmSystem("test up");
    //public final CoralClaw coralClawSystem = new CoralClaw();

    //private final CoralArmTeleOpCommand runElevatorUp = new CoralArmTeleOpCommand(coralArmSystem, 1);
    //private final CoralArmTeleOpCommand runElevatorDown = new CoralArmTeleOpCommand(coralArmSystem, -1);
    //DoubleSupplier sup = () -> operatorJoystick.getRightY();
    //public final CoralArmSystem coralArmSystem = new CoralArmSystem(sup);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        if (Settings.useFlightStick) {

            drivingJoystickProfile = new DrivingJoystickProfile(
                () -> driverFlightHotasOne.getY(), 
                () -> driverFlightHotasOne.getX(), 
                () -> -driverFlightHotasOne.getTwist(), 
                () -> 0.5 + 0.5 * driverFlightHotasOne.getThrottle(), 
                2, 3);

            drivetrain.setDefaultCommand( // Flight Stick
            // Drivetrain will execute this command periodically
                // normal
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(drivingJoystickProfile.getStrafeOutput() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(drivingJoystickProfile.getForwardOutput() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(drivingJoystickProfile.getRotationOutput() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
                
            );

            // Old Hotas
            /* 
            new JoystickButton(driverFlightHotasOne, 7).whileTrue(drivetrain.applyRequest(() -> brake));
            new JoystickButton(driverFlightHotasOne, 6).whileTrue(drivetrain.applyRequest(() ->
                point.withModuleDirection(new Rotation2d(-driverFlightHotasOne.getY(), -driverFlightHotasOne.getX()))
            ));

            // reset the field-centric heading on left bumper press
            new JoystickButton(driverFlightHotasOne, 5).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
            new JoystickButton(driverFlightHotasOne, 14).onTrue(new InstantCommand(climbingSystem::StartClimbing)).onFalse(new InstantCommand(climbingSystem::StopClimbing));
            new JoystickButton(driverFlightHotasOne, 13).onTrue(new InstantCommand(climbingSystem::StartUnclimbing)).onFalse(new InstantCommand(climbingSystem::StopClimbing));
            */



            // new hotas
            new JoystickButton(driverFlightHotasOne, 7).whileTrue(drivetrain.applyRequest(() -> brake));
            new JoystickButton(driverFlightHotasOne, 6).whileTrue(drivetrain.applyRequest(() ->
                point.withModuleDirection(new Rotation2d(driverFlightHotasOne.getY(), driverFlightHotasOne.getX()))
            ));

            // reset the field-centric heading on left bumper press
            new JoystickButton(driverFlightHotasOne, 5).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
            new JoystickButton(driverFlightHotasOne, 14).onTrue(new InstantCommand(climbingSystem::StartClimbing)).onFalse(new InstantCommand(climbingSystem::StopClimbing));
            new JoystickButton(driverFlightHotasOne, 13).onTrue(new InstantCommand(climbingSystem::StartUnclimbing)).onFalse(new InstantCommand(climbingSystem::StopClimbing));



        } else { // Normal Logitech or xbox Controller
            /* 
            drivingProfile = new DrivingProfile(
                () -> -driverJoystick.getLeftY(), 
                () -> -driverJoystick.getLeftX(), 
                () -> -driverJoystick.getRightX(), 
                () -> 0.5 + 0.5 * driverJoystick.getLeftTriggerAxis(), 
                3, 3, 5);
                */

            drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
                // normal
                /*
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(drivingProfile.getStrafeOutput() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(drivingProfile.getForwardOutput() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(drivingProfile.getRotationOutput() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
                        */

                // old
                
                // i am being very safe with this
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed * (0.5 + 0.5 * Math.abs(driverJoystick.getRightTriggerAxis()))) // Drive forward with negative Y (forward)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed * (0.5 + 0.5 * Math.abs(driverJoystick.getRightTriggerAxis()))) // Drive left with negative X (left)
                        .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate * (0.6 + 0.4 * Math.abs(driverJoystick.getRightTriggerAxis()))) // Drive counterclockwise with negative X (left)
                )
                
            );

            driverJoystick.b().whileTrue(drivetrain.applyRequest(() -> brake));
            driverJoystick.a().whileTrue(drivetrain.applyRequest(() ->
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

            //driverJoystick.povUp().onTrue(new InstantCommand(climbingSystem::StartUnclimbing)).onFalse(new InstantCommand(climbingSystem::StopClimbing));
            //driverJoystick.povDown().onTrue(new InstantCommand(climbingSystem::StartClimbing)).onFalse(new InstantCommand(climbingSystem::StopClimbing));

        }

        //operatorJoystick.leftBumper().onTrue(runElevatorUp);
        //operatorJoystick.leftTrigger().onTrue(runElevatorDown);
        //operatorJoystick.rightBumper().onTrue(new InstantCommand(coralClawSystem::outtakeRoller)).onFalse(new InstantCommand(coralClawSystem::stopRoller));
        //operatorJoystick.rightTrigger().onTrue(new InstantCommand(coralClawSystem::intakeRoller)).toggleOnFalse(new InstantCommand(coralClawSystem::holdRoller));
        operatorJoystick.rightBumper().onTrue(new InstantCommand(algaeArmSystem::outtakeRollerClaw)).onFalse(new InstantCommand(algaeArmSystem::stopRollerClaw));
        operatorJoystick.rightTrigger().onTrue(new InstantCommand(algaeArmSystem::intakeRollerClaw)).toggleOnFalse(new InstantCommand(algaeArmSystem::holdRollerClaw));
        
        //operatorJoystick.povDown().onTrue(new InstantCommand(algaeArmSystem::toggleRightBias));
        //operatorJoystick.povUp().onTrue(new InstantCommand(coralArmSystem::toggleCoralArm));

        operatorJoystick.b().onTrue(new InstantCommand(algaeArmSystem::realignAlgaeArm));
        operatorJoystick.y().onTrue(new InstantCommand(algaeArmSystem::enableArm));
        operatorJoystick.x().onTrue(new InstantCommand(algaeArmSystem::stopArm));

        operatorJoystick.povDown().onTrue(new InstantCommand(algaeArmSystem::presetFloorForward));
        operatorJoystick.povUp().onTrue(new InstantCommand(algaeArmSystem::presetStowInCenter));

        //operatorJoystick.povDownLeft().onTrue(new InstantCommand(coralClawSystem::toggleLeftDiffyPosition));
        //operatorJoystick.povDownRight().onTrue(new InstantCommand(coralArmSystem::toggleStoreArm));

        // temporary Algae Arm controls
        algaeArmSystem.setControllerInputsJoint( 
            () -> operatorJoystick.getLeftY() * AlgaeArmSettings.maxJoystickMovementSpeed, 
            () -> operatorJoystick.getRightY() * AlgaeArmSettings.maxJoystickMovementSpeed
        );

        //coralClawSystem.setDefaultCommand(new RunCommand(coralClawSystem::update, coralClawSystem));
        //coralArmSystem.setDefaultCommand(new RunCommand(coralArmSystem::update, coralArmSystem));
        algaeArmSystem.setDefaultCommand(new RunCommand(algaeArmSystem::updateInTeleOp, algaeArmSystem));
        //climbingSystem.setDefaultCommand(new RunCommand(climbingSystem::update, climbingSystem));
        
        //Coral Elevator Controls
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
