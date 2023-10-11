package team.ironriders.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team.ironriders.constants.Constants;
import team.ironriders.subsystems.ArmSubsystem;
import team.ironriders.subsystems.DriveSubsystem;
import team.ironriders.subsystems.ManipulatorSubsystem;

import static team.ironriders.constants.Commands.*;

public class AutoCommands {
    public enum AutoMode {
        CUBE_L1,
        CUBE_L2,
        CUBE_L3
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
            case CUBE_L1 -> placeL1();
            case CUBE_L2 -> placeL2();
            case CUBE_L3 -> placeL3();
            default -> null;
        };
    }

    private Command placeL1() {
        return Cube.L1(arm)
                .andThen(MAN_R(manipulator))
                .andThen(retract());
    }

    private Command placeL2() {
        return Cube.L2(arm)
                .andThen(MAN_R(manipulator))
                .andThen(retract());
    }

    private Command placeL3() {
        return Cube.L3(arm)
                .andThen(MAN_R(manipulator))
                .andThen(retract());
    }

    private Command retract() {
        return new ClimberCommand(30, arm)
                .andThen(R(arm))
                .alongWith(new MecanumPathFollower(drive, "test", Constants.SlowAutoConstraints, true));
    }
}
