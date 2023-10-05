// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.ironriders;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import team.ironriders.subsystems.ArmSubsystem;
import team.ironriders.subsystems.DriveSubsystem;
import team.ironriders.subsystems.LightsSubsystem;
import team.ironriders.subsystems.ManipulatorSubsystem;

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
    SendableChooser<String> speedChooser = new SendableChooser<>();
    String lastSelectedSpeed = String.valueOf(Constants.DRIVE_SPEED_SLOW);

    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    public void periodic() {
        if (!lastSelectedSpeed.equals(speedChooser.getSelected())) {
            speedMultiplier = Double.parseDouble(speedChooser.getSelected());
            lastSelectedSpeed = String.valueOf(speedMultiplier);
        }

        if (SmartDashboard.getBoolean("Reset Encoders", false)) {
            SmartDashboard.putBoolean("Reset Encoders", false);
            arm.resetEncoders();
            drive.resetPigeon();
            manipulator.resetEncoders();
        }
        SmartDashboard.putNumber("Volts", pdh.getVoltage());
        lights.setChassisSpeeds(drive.getChassisSpeeds());
    }


    /** Use this method to define your trigger->command mappings. */
    private void configureBindings() {
        speedChooser.setDefaultOption("Default", "0.1");
        for (double i = 0.2; i < 1; i += 0.1) {
            speedChooser.addOption(String.format("Speed %.1f", i), String.valueOf(i));
        }
        SmartDashboard.putData(speedChooser);

        drive.setDefaultCommand(
                new RunCommand(
                        () ->
                                drive.setChassisSpeeds(
                                        -scaledDeadBand(controller.getX()),
                                        -scaledDeadBand(controller.getY()),
                                        -scaledDeadBand(controller.getTwist() * 0.8),
                                        false),
                        drive));

        controller.button(3).whileTrue(new StartEndCommand(arm::lower, arm::stopPivot, arm));
        controller.button(5).whileTrue(new StartEndCommand(arm::raise, arm::stopPivot, arm));

        controller.povDown().whileTrue(new StartEndCommand(arm::retract, arm::stopClimber, arm));
        controller.povUp().whileTrue(new StartEndCommand(arm::extend, arm::stopClimber, arm));

        controller.button(4).whileTrue(new StartEndCommand(manipulator::grab, manipulator::stop, manipulator));
        controller.button(6).whileTrue(new StartEndCommand(manipulator::release, manipulator::stop, manipulator));

        controller.button(9).onTrue(Commands.runOnce(() -> pivotClimberPreset(1.3, 220)));
        controller.button(10).onTrue(Commands.runOnce(() -> pivotClimberPreset(0, 1)));

        controller.button(11).onTrue(Commands.runOnce(() -> arm.setClimber(100), arm));
        controller.button(12).onTrue(Commands.runOnce(() -> arm.setClimber(0), arm));

        controller.button(8).onTrue(
                Commands.runOnce(() -> speedMultiplier = Constants.DRIVE_SPEED_FAST)
        ).onFalse(
                Commands.runOnce(() -> speedMultiplier = Constants.DRIVE_SPEED_SLOW)
        );
        controller.button(1).onTrue(
                Commands.runOnce(() -> speedMultiplier = Constants.DRIVE_SPEED_FAST)
        ).onFalse(
                Commands.runOnce(() -> speedMultiplier = Constants.DRIVE_SPEED_SLOW)
        );
    }

    private double scaledDeadBand(double value) {
        double value1 = MathUtil.applyDeadband(value, Constants.DEADBAND);
        return Math.signum(value1) * Math.pow(Math.abs(value1), 1) * speedMultiplier;
    }

    private void pivotClimberPreset(double pivot, double climber) {
        arm.setPivot(pivot);
        arm.setClimber(climber);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // TODO: Implement properly
        return null;
    }
}
