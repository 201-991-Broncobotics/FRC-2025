package frc.robot.commands;

import frc.robot.subsystems.AlgaeArmSystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeArmTeleOpCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AlgaeArmSystem algaeArmSystem;


    public AlgaeArmTeleOpCommand(AlgaeArmSystem subsystem) {
        algaeArmSystem = subsystem;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
