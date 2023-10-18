package team.ironriders.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.ironriders.constants.Constants;
import team.ironriders.subsystems.DriveSubsystem;


public class RotateToCommand extends CommandBase {
    private final double target;
    private final DriveSubsystem drive;

    public RotateToCommand(double target, DriveSubsystem drive) {
        this.target = target;
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        // test dividing error

        drive.setChassisSpeeds(
                0,
                0,
                MathUtil.clamp(-error() / 20, -1, 1) * Constants.DRIVE_SPEED_SLOW,
                false
        );
    }

    @Override
    public boolean isFinished() {
        return Math.abs(error()) < 3;
    }

    private double error() {
        double error = (target - drive.pigeon.getAngle() % 360) % 360;
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }
        return error;
    }
}
