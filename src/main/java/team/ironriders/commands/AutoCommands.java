package team.ironriders.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team.ironriders.subsystems.ArmSubsystem;
import team.ironriders.subsystems.DriveSubsystem;
import team.ironriders.subsystems.ManipulatorSubsystem;

import static team.ironriders.commands.PivotClimberMoveCommand.Preset.*;

public class AutoCommands {
    public enum AutoMode {
        PLACE_GAME_PIECE_L1,
        PLACE_GAME_PIECE_L2,
        PLACE_GAME_PIECE_L3
    }

    ArmSubsystem arm;
    ManipulatorSubsystem manipulator;
    DriveSubsystem drive;

    public Command getCommand(AutoMode autoMode, ArmSubsystem arm, ManipulatorSubsystem manipulator,
                              DriveSubsystem drive) {
        this.arm = arm;
        this.manipulator = manipulator;
        this.drive = drive;

        return switch (autoMode) {
            case PLACE_GAME_PIECE_L1 -> placeL1();
            case PLACE_GAME_PIECE_L2 -> placeL2();
            case PLACE_GAME_PIECE_L3 -> placeL3();
            default -> null;
        };
    }

    private Command placeL1() {
        return new PivotClimberMoveCommand(L1, arm)
                .withTimeout(5)
                .andThen(new ManipulatorCommand(ManipulatorCommand.Direction.RELEASE, manipulator));
    }

    private Command placeL2() {
        return new PivotClimberMoveCommand(L2, arm)
                .withTimeout(5)
                .andThen(new ManipulatorCommand(ManipulatorCommand.Direction.RELEASE, manipulator));
    }

    private Command placeL3() {
        return new PivotClimberMoveCommand(L3, arm)
                .withTimeout(5)
                .andThen(new ManipulatorCommand(ManipulatorCommand.Direction.RELEASE, manipulator));
    }
}
