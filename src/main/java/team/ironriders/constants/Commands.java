package team.ironriders.constants;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team.ironriders.commands.ClimberCommand;
import team.ironriders.commands.ManipulatorCommand;
import team.ironriders.commands.PivotClimberMoveCommand;
import team.ironriders.commands.PivotCommand;
import team.ironriders.subsystems.ArmSubsystem;
import team.ironriders.subsystems.ManipulatorSubsystem;

public class Commands {
    public static CommandBase R(ArmSubsystem arm) {
        return new PivotClimberMoveCommand(0.5, 30, arm);
    }

    public static CommandBase HP(ArmSubsystem arm, ManipulatorSubsystem manipulator) {
        return new PivotClimberMoveCommand(1.475, 80, arm)
                .andThen(new ManipulatorCommand(ManipulatorCommand.Direction.RELEASE, manipulator));
    }

    public static CommandBase MAN_R(ManipulatorSubsystem manipulator) {
        return new ManipulatorCommand(ManipulatorCommand.Direction.RELEASE, manipulator)
                .withTimeout(1.3);
    }

    public static class Cone {
        public static CommandBase L1(ArmSubsystem arm) {
            return new PivotClimberMoveCommand(0.6, 65, arm);
        }

        public static CommandBase L2(ArmSubsystem arm) {
            return new ClimberCommand(20, arm)
                    .andThen(new PivotCommand(1.43, arm))
                    .andThen(new ClimberCommand(60, arm));
        }

        public static CommandBase L3(ArmSubsystem arm) {
            return new ClimberCommand(20, arm)
                    .andThen(new PivotCommand(1.7, arm))
                    .andThen(new ClimberCommand(220, arm));
        }
    }

    public static class Cube {
        public static CommandBase L1(ArmSubsystem arm) {
            return Commands.Cone.L1(arm);
        }

        public static CommandBase L2(ArmSubsystem arm) {
            return new ClimberCommand(0, arm)
                    .andThen(new PivotCommand(1.25, arm))
                    .andThen(new ClimberCommand(110, arm));
        }

        public static CommandBase L3(ArmSubsystem arm) {
            return new ClimberCommand(0, arm)
                    .andThen(new PivotCommand(1.55, arm))
                    .andThen(new ClimberCommand(220, arm));
        }
    }
}
