package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevatorSystem;

public class StowElevatorCommand extends Command {
    private final CoralElevatorSystem coralElevatorSystem;

    public StowElevatorCommand(CoralElevatorSystem elevatorSystem) {
        coralElevatorSystem = elevatorSystem;
        addRequirements(coralElevatorSystem); // Declare subsystem requirements
    }

    @Override
    public void initialize() {
        coralElevatorSystem.ActuallyGoToStow();
    }

    @Override
    public void execute() {
        // Called repeatedly while the command is scheduled.
        coralElevatorSystem.update();
    }

    @Override
    public boolean isFinished() {
        return coralElevatorSystem.isFinishedMoving(); // Or false if it's a continuous action that needs a separate stop command
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends or is interrupted.
    }
}