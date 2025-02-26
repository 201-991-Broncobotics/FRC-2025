package frc.robot.commands;

import frc.robot.subsystems.CoralArmSystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralArmTeleOpCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final CoralArmSystem coralArmSystem;
    private final int direction;


    public CoralArmTeleOpCommand(CoralArmSystem subsystem, int direction) {
        this.direction =direction;
        coralArmSystem=subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(this.direction>0&coralArmSystem.getStage()<3) coralArmSystem.setStage(coralArmSystem.getStage()+1);
        if(this.direction<0&coralArmSystem.getStage()>0) coralArmSystem.setStage(coralArmSystem.getStage()-1);
        switch (coralArmSystem.getStage()) {
            case 0:
                coralArmSystem.setElevatorPos(0);
                break;
            case 1:
                coralArmSystem.setElevatorPos(20);
                break;
            case 2:
                coralArmSystem.setElevatorPos(60);
                break;
             default:
                break;
        }
        SmartDashboard.putNumber("Elevator Stage", coralArmSystem.getStage());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
