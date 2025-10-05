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
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralElevatorSystem;
import frc.robot.subsystems.DrivingProfiles;
import frc.robot.utility.Functions;

public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 10% deadband - now 6%
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);
    private final Joystick driverFlightHotasOne = new Joystick(2);

    // private final Vision vision = new Vision();

    

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final AlgaeArm algaeArm = new AlgaeArm();
    //public final ClimbingSystem climbingSystem = new ClimbingSystem();

    public final CoralElevatorSystem coralElevatorSystem = new CoralElevatorSystem();

    private final DrivingProfiles drivingProfile = new DrivingProfiles(drivetrain);

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
        
        drivetrain.configureAutoBuilder();
        //selecting the pathplanner auto you want from dashboard + setting default
        autoChooser = AutoBuilder.buildAutoChooser("middleAuto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        /* 
        try {
            CameraServer.startAutomaticCapture("Limelight", "http://limelight.local:5800/stream.mjpg");
            SmartDashboard.putString("Limelight:",  "WORKING, in theory");
        } catch (Exception e) {
            // System.out.println("Hey, I'm not working");
            SmartDashboard.putString("Limelight:",  "Hey, I'm not working");
        } */

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
        ); 

            
        // DRIVER CONTROLS

        // Auto Targeting
        // drivingProfile.setUpAutoThrottleControllerInput(() -> driverJoystick.getLeftTriggerAxis());
        // drivingProfile.setUpAutoThrottleJoystickInput(() -> 0.2 + 0.8 * ((-driverFlightHotasOne.getRawAxis(2)+1)/2));


        // AUTO DRIVING
        drivingProfile.setupAutoDrivingThrottle(() -> driverJoystick.getLeftTriggerAxis()); 
        // new JoystickButton(driverFlightHotasOne, 15).onTrue(new InstantCommand(drivingProfile::enableAutoDriving)).onFalse(new InstantCommand(drivingProfile::disableAutoDriving));

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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        




        // OPERATOR CONTROLS

        coralElevatorSystem.setManualControl(() -> ((operatorJoystick.leftBumper().getAsBoolean())? 0.5:0.0) + (-Functions.deadbandValue(operatorJoystick.getLeftTriggerAxis(), 0.1))); //-Functions.deadbandValue(operatorJoystick.getLeftY(),  0.1)); // 
        coralElevatorSystem.setManualPivotControl(() -> -Functions.deadbandValue(operatorJoystick.getLeftY(),  0.05));
        operatorJoystick.povDown().toggleOnTrue(new InstantCommand(coralElevatorSystem::JumpElevatorToPickup));
        operatorJoystick.povLeft().toggleOnTrue(new InstantCommand(coralElevatorSystem::goToFunnel));
        operatorJoystick.povUp().toggleOnTrue(new InstantCommand(coralElevatorSystem::goToL4));
        operatorJoystick.povRight().toggleOnTrue(new InstantCommand(coralElevatorSystem::goToStow));

        //operatorJoystick.x().toggleOnTrue(new InstantCommand(coralElevatorSystem::ArmToggleEnabled));

        //operatorJoystick.povLeft().onTrue(new InstantCommand(() -> coralClaw.goToPreset(CoralSystemPresets.GroundIntake))).onTrue(new InstantCommand(() -> coralElevatorSystem.goToPreset(CoralSystemPresets.GroundIntake)));
        //operatorJoystick.povUp().toggleOnTrue(new InstantCommand(coralElevatorSystem::upOneStage)).toggleOnFalse(new InstantCommand(coralClaw::goToElevatorPreset)).toggleOnFalse(new InstantCommand(coralElevatorSystem::stopChangingStage));
        //operatorJoystick.povDown().toggleOnTrue(new InstantCommand(coralElevatorSystem::downOneStage)).toggleOnFalse(new InstantCommand(coralClaw::goToElevatorPreset)).toggleOnFalse(new InstantCommand(coralElevatorSystem::stopChangingStage));
        //operatorJoystick.povLeft().toggleOnTrue(new InstantCommand(coralClaw::switchRotation));
        //operatorJoystick.povRight().toggleOnTrue(new InstantCommand(coralElevatorSystem::goToCoralStationPreset)).toggleOnFalse(new InstantCommand(coralClaw::goToElevatorPreset));
        
        // Algae
        algaeArm.setManualControl(() -> -Functions.deadbandValue(operatorJoystick.getRightY(),  0.1)); //algaeArm.setManualControl(() -> (operatorJoystick.povLeft().getAsBoolean())? 0.5:0 + -operatorJoystick.getLeftTriggerAxis());
        operatorJoystick.rightBumper().onTrue(new InstantCommand(algaeArm::outtakeRoller)).toggleOnFalse(new InstantCommand(algaeArm::stopRoller));
        operatorJoystick.rightTrigger().onTrue(new InstantCommand(algaeArm::intakeRoller)).toggleOnFalse(new InstantCommand(algaeArm::holdRoller));

        //operatorJoystick.x().onTrue(new InstantCommand(algaeArm::presetOuttakePosition));
        //operatorJoystick.y().onTrue(new InstantCommand(algaeArm::presetStorePosition));
        //operatorJoystick.b().onTrue(new InstantCommand(algaeArm::presetIntakePosition));
        //operatorJoystick.a().toggleOnTrue(new InstantCommand(coralClaw::toggleEnabled));

        coralElevatorSystem.setDefaultCommand(new RunCommand(coralElevatorSystem::update, coralElevatorSystem));
        algaeArm.setDefaultCommand(new RunCommand(algaeArm::update, algaeArm));
        
        
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
