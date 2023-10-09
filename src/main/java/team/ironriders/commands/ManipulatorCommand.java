package team.ironriders.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.ironriders.lib.Utils;
import team.ironriders.subsystems.ManipulatorSubsystem;


public class ManipulatorCommand extends CommandBase {
    public enum Direction {
        GRAB,
        RELEASE
    }

    Direction direction;
    ManipulatorSubsystem manipulator;

    /**
     * Direction.GRAB is not ready yet
     */
    public ManipulatorCommand(Direction direction, ManipulatorSubsystem manipulator) {
        this.direction = direction;
        this.manipulator = manipulator;

        addRequirements(manipulator);
    }

    @Override
    public void execute() {
        manipulator.setManipulatorMotors(direction.equals(Direction.RELEASE) ? -1 : 1);
    }

    @Override
    public boolean isFinished() {
        double target = direction.equals(Direction.RELEASE) ? -26.5 : 0;
        return Utils.isWithinTolerance(manipulator.getRightPos(), target, 1) &&
                Utils.isWithinTolerance(manipulator.getLeftPos(), target, 1);
    }

    @Override
    public void end(boolean interrupted) {
        manipulator.setManipulatorMotors(0);
    }
}
