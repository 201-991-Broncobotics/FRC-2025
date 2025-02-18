package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSystem;
import frc.robot.utility.Vector2d;

public class SwitchMiddleJointSides extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final AlgaeArmSystem algaeArmSystem;

    private boolean isOnNewSide = false;

    private Vector2d initialTarget;

    public SwitchMiddleJointSides(AlgaeArmSystem subsystem) {
        algaeArmSystem = subsystem;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialTarget = algaeArmSystem.getMovingTarget();
        algaeArmSystem.setTargetMaxInDirection(algaeArmSystem.getMovingTarget().angle());
        isOnNewSide = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (algaeArmSystem.isCloseToTarget()) {
            algaeArmSystem.setRightBias(!algaeArmSystem.getRightBias());
            algaeArmSystem.setTarget(initialTarget);
            isOnNewSide = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isOnNewSide && algaeArmSystem.isCloseToTarget();
    }


}
