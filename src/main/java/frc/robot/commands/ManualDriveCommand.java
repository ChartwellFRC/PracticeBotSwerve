package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Teleop manual drive command for the swerve drivetrain.
 */
public class ManualDriveCommand extends Command {
    private final DriveSubsystem drive;
    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
    private final DoubleSupplier rotationInput;

    public ManualDriveCommand(
        DriveSubsystem drive,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput,
        DoubleSupplier rotationInput
    ) {
        this.drive = drive;
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
        this.rotationInput = rotationInput;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(forwardInput.getAsDouble(), leftInput.getAsDouble(), rotationInput.getAsDouble(), false);
    }

    @Override
    public boolean isFinished() {
        // Default drive command: runs until interrupted
        return false;
    }
}
