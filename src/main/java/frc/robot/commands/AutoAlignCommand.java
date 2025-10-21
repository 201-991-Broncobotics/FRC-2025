package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivingProfiles;
import frc.robot.utility.PeriodicWaitTimer;

public class AutoAlignCommand extends Command {
    private final DrivingProfiles drivingProfile;

    private PeriodicWaitTimer TimeoutTimer;
    private boolean shouldBeFinished;

    public AutoAlignCommand(DrivingProfiles DrivingProfile) {
        drivingProfile = DrivingProfile;
        TimeoutTimer = new PeriodicWaitTimer(); 
        addRequirements(drivingProfile); // Declare subsystem requirements
    }

    @Override
    public void initialize() {
        drivingProfile.enableAutoDriving();
        shouldBeFinished = false;
    }

    @Override
    public void execute() {
        // Called repeatedly while the command is scheduled.
        drivingProfile.updateAutoDriving();
        if ((drivingProfile.getAutoDrivePower() < 0.075) && (drivingProfile.getAutoDriveTurnPower() < 0.075)) {
            TimeoutTimer.start(5);
        }
        if (TimeoutTimer.isPastTime()) {
            shouldBeFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return true; // Or false if it's a continuous action that needs a separate stop command
    }

    @Override
    public void end(boolean interrupted) {
        // Called once when the command ends or is interrupted.
        drivingProfile.disableAutoDriving();
        drivingProfile.stopDriving();
    }
}