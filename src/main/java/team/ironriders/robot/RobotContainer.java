// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.ironriders.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import team.ironriders.commands.AutoCommands;
import team.ironriders.constants.Constants;
import team.ironriders.subsystems.ArmSubsystem;
import team.ironriders.subsystems.DriveSubsystem;
import team.ironriders.subsystems.LightsSubsystem;
import team.ironriders.subsystems.ManipulatorSubsystem;

import static team.ironriders.constants.Commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final LightsSubsystem lights = new LightsSubsystem();
    public final ArmSubsystem arm = new ArmSubsystem();
    public final DriveSubsystem drive = new DriveSubsystem();
    public final ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
    private final CommandJoystick controller = new CommandJoystick(0);
    private double speedMultiplier = Constants.DRIVE_SPEED_SLOW;
    PowerDistribution pdh = new PowerDistribution(13, PowerDistribution.ModuleType.kRev);
    SendableChooser<String> autoSelector = new SendableChooser<>();
    SendableChooser<String> leaveCommunity = new SendableChooser<>();

    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    public void periodic() {
        SmartDashboard.putNumber("Volts", pdh.getVoltage());
    }


    /** Use this method to define your trigger->command mappings. */
    private void configureBindings() {
        autoSelector.setDefaultOption("Cube L3", "CUBE_L3");
        autoSelector.addOption("Cube L2", "CUBE_L2");
        autoSelector.addOption("Cube L1", "CUBE_L1");
        autoSelector.addOption("Cone L3", "CONE_L3");
        autoSelector.addOption("Cone L2", "CONE_L2");
        autoSelector.addOption("Cone L1", "CONE_L1");
        SmartDashboard.putData(autoSelector);

        leaveCommunity.setDefaultOption("True", "true");
        leaveCommunity.addOption("False", "false");
        SmartDashboard.putData(leaveCommunity);

        drive.setDefaultCommand(
                new RunCommand(
                        () ->
                                drive.setChassisSpeeds(
                                        -scaledDeadBand(controller.getX()),
                                        -scaledDeadBand(controller.getY()),
                                        -scaledDeadBand(controller.getTwist() * 0.8),
                                        false),
                        drive));

        // pivot (up, down)
        controller.button(3).whileTrue(new StartEndCommand(arm::lower, arm::stopPivot, arm));
        controller.button(5).whileTrue(new StartEndCommand(arm::raise, arm::stopPivot, arm));

        // climber (in, out)
        controller.povDown().whileTrue(new StartEndCommand(arm::retract, arm::stopClimber, arm));
        controller.povUp().whileTrue(new StartEndCommand(arm::extend, arm::stopClimber, arm));

        // manipulator (in, out)
        controller.button(4).whileTrue(new StartEndCommand(manipulator::grab, manipulator::stop, manipulator));
        controller.button(6).whileTrue(new StartEndCommand(manipulator::release, manipulator::stop, manipulator));

        // l3
        controller.button(7).onTrue(Cube.L3(arm));
        // l2
        controller.button(10).onTrue(Cube.L2(arm));
        // l1
        controller.button(11).onTrue(Cone.L1(arm));

        // human player
        controller.button(8).onTrue(HP(arm, manipulator));
        // resting
        controller.button(12).onTrue(R(arm));

        // boost
        controller.button(1).onTrue(
                Commands.runOnce(() -> speedMultiplier = Constants.DRIVE_SPEED_FAST)
        ).onFalse(
                Commands.runOnce(() -> speedMultiplier = Constants.DRIVE_SPEED_SLOW)
        );
        // boost alt
        controller.button(9).onTrue(
                Commands.runOnce(() -> speedMultiplier = Constants.DRIVE_SPEED_FAST)
        ).onFalse(
                Commands.runOnce(() -> speedMultiplier = Constants.DRIVE_SPEED_SLOW)
        );
    }

    private double scaledDeadBand(double value) {
        double value1 = MathUtil.applyDeadband(value, Constants.DEADBAND);
        return Math.signum(value1) * Math.pow(Math.abs(value1), 1) * speedMultiplier;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Cube L3 leave community gives us the most points with the highest success rate
        return new AutoCommands().getCommand(
                AutoCommands.AutoModePiece.valueOf(autoSelector.getSelected()),
                Boolean.parseBoolean(leaveCommunity.getSelected()),
                arm,
                manipulator,
                drive
        );
    }
}
