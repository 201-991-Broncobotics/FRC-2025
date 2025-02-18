package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSystem;
import frc.robot.utility.Vector2d;

public class DeployAlgaeArm extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final AlgaeArmSystem algaeArmSystem;

    private final Vector2d FirstTarget = new Vector2d(5, 40);
    private final double resetAngle = Math.toRadians(90);
    private final Vector2d FinalTarget = new Vector2d(-12, 0);

    private int stage = 0;

    public DeployAlgaeArm(AlgaeArmSystem subsystem) {
        algaeArmSystem = subsystem;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stage = 0;
        algaeArmSystem.setTarget(FirstTarget);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switch (stage) {
            case 0:
                if (algaeArmSystem.isCloseToTarget()) {
                    stage = 1;
                    algaeArmSystem.setTargetMaxInDirection(resetAngle);
                }
                break;
            case 1:
                if (algaeArmSystem.isCloseToTarget()) {
                    stage = 2;
                    algaeArmSystem.setRightBias(false);
                    algaeArmSystem.setTarget(FinalTarget);
                }
                break;
        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return stage == 2 && algaeArmSystem.isCloseToTarget();
    }



}
