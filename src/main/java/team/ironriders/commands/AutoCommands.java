package team.ironriders.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team.ironriders.constants.Constants;
import team.ironriders.subsystems.ArmSubsystem;
import team.ironriders.subsystems.DriveSubsystem;
import team.ironriders.subsystems.ManipulatorSubsystem;

import static team.ironriders.constants.Commands.*;

public class AutoCommands {
    public enum AutoModePiece {
        CUBE_L1,
        CUBE_L2,
        CUBE_L3,
        CONE_L1,
        CONE_L2,
        CONE_L3
    }

    ArmSubsystem arm;
    ManipulatorSubsystem manipulator;
    DriveSubsystem drive;
    boolean leaveCommunity;

    public Command getCommand(AutoModePiece autoMode, boolean leaveCommunity, ArmSubsystem arm,
                              ManipulatorSubsystem manipulator, DriveSubsystem drive) {
        this.arm = arm;
        this.manipulator = manipulator;
        this.drive = drive;
        this.leaveCommunity = leaveCommunity;

        return switch (autoMode) {
            case CUBE_L1 -> genericPlace(Cube.L1(arm));
            case CUBE_L2 -> genericPlace(Cube.L2(arm));
            case CUBE_L3 -> genericPlace(Cube.L3(arm));
            case CONE_L1 -> genericPlace(Cone.L1(arm));
            case CONE_L2 -> genericPlace(Cone.L2(arm));
            case CONE_L3 -> genericPlace(Cone.L3(arm));
        };
    }

    private Command genericPlace(CommandBase command) {
        return command
                .andThen(MAN_R(manipulator))
                .andThen(retract());
    }

    private Command retract() {
        return new ClimberCommand(30, arm)
                .andThen(new PivotClimberMoveCommand(0, 0, arm))
                .alongWith(
                        new MecanumPathFollower(drive, "test", Constants.SlowAutoConstraints, true)
                                .unless(() -> !leaveCommunity)
                );
    }
}
