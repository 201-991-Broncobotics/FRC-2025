package frc.robot.commands;

import frc.robot.subsystems.CoralArmSystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralArmTeleOpCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final CoralArmSystem coralArmSystem;
    public static int currentPosition=0;
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
        if(this.direction>0&currentPosition<3); currentPosition+=1;
        if(this.direction<0&currentPosition>0); currentPosition-=1;
        switch (currentPosition) {
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
        SmartDashboard.putNumber("Elevator Stage", currentPosition);
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
