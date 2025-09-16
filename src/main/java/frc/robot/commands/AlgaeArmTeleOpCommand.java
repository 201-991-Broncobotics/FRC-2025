package frc.robot.commands;

import frc.robot.subsystems.AlgaeArm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeArmTeleOpCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final AlgaeArm algaeArmSystem;

    private DoubleSupplier ArmVerticalAxisControl, ArmHorizontalAxisControl;


    public AlgaeArmTeleOpCommand(AlgaeArm subsystem, DoubleSupplier verticalAxisControl, DoubleSupplier horizontalAxisControl) {
        algaeArmSystem = subsystem;
        ArmVerticalAxisControl = verticalAxisControl;
        ArmHorizontalAxisControl = horizontalAxisControl;

        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // algaeArmSystem.setTargetMoveSpeeds(ArmVerticalAxisControl.getAsDouble(), ArmHorizontalAxisControl.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
