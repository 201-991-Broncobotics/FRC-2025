package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevatorSystem;
import frc.robot.utility.PeriodicWaitTimer;

public class PlaceL4CoralCommand extends Command {
    private final CoralElevatorSystem coralElevatorSystem;

    private PeriodicWaitTimer DepositTimeout;
    private boolean shouldBeFinished;

    public PlaceL4CoralCommand(CoralElevatorSystem elevatorSystem) {
        coralElevatorSystem = elevatorSystem;
        addRequirements(coralElevatorSystem); // Declare subsystem requirements
    }

    @Override
    public void initialize() {
        shouldBeFinished = false;
        coralElevatorSystem.DepositOnL4();
        DepositTimeout = new PeriodicWaitTimer(); 
        DepositTimeout.start(1);
    }

    @Override
    public void execute() {
        // Called repeatedly while the command is scheduled.
        coralElevatorSystem.update();
        if (DepositTimeout.isPastTime()) {
            coralElevatorSystem.goToAfterL4();
            coralElevatorSystem.update();
            shouldBeFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return shouldBeFinished && coralElevatorSystem.isFinishedMoving(); // Or false if it's a continuous action that needs a separate stop command
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends or is interrupted.
    }
}