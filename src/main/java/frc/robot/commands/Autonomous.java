package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utility.ElapsedTime;

public class Autonomous extends Command {

    private ElapsedTime timer;

    private CommandSwerveDrivetrain drivetrain;

    private double MaxSpeed = RobotContainer.MaxSpeed; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RobotContainer.MaxAngularRate; // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public Autonomous(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        //drivetrain.setControl(drive.withVelocityX(-0.5 * MaxSpeed).withVelocityY(0.0).withRotationalRate(0.0));
    }

    @Override
    public void execute() {
        //drivetrain.applyRequest(() -> );
        drivetrain.setControl(drive.withVelocityX(-0.25 * MaxSpeed).withVelocityY(0.0).withRotationalRate(0.0));
    }

    @Override
    public void end(boolean interrupted) {
        //drivetrain.applyRequest(() -> drive.withVelocityX(0.0));
    }

    @Override
    public boolean isFinished() {
        return timer.time() > 0.5;
    }

}
