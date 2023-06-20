// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.ironriders;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
    private double speedMultiplier = 0.5;
    PowerDistribution pdh = new PowerDistribution(13, PowerDistribution.ModuleType.kRev);
    SendableChooser<String> speedChooser = new SendableChooser<>();

    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        SmartDashboard.putBoolean("Reset Encoders", false);
    }

    public void periodic() {
        speedMultiplier = Double.parseDouble(speedChooser.getSelected());

        if (SmartDashboard.getBoolean("Reset Encoders", false)) {
            SmartDashboard.putBoolean("Reset Encoders", false);
            arm.resetEncoders();
        }
        SmartDashboard.putNumber("Volts", pdh.getVoltage());

        lights.setChassisSpeeds(drive.getChassisSpeeds());
    }


    /** Use this method to define your trigger->command mappings. */
    private void configureBindings() {
        speedChooser.setDefaultOption("Speed 0.1", "0.1");
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
                                        -scaledDeadBand(controller.getTwist()),
                                        false),
                        drive));

        controller.button(3).whileTrue(new StartEndCommand(arm::lower, arm::stopPivot, arm));
        controller.button(5).whileTrue(new StartEndCommand(arm::raise, arm::stopPivot, arm));

        controller.povDown().whileTrue(new StartEndCommand(arm::retract, arm::stopClimber, arm));
        controller.povUp().whileTrue(new StartEndCommand(arm::extend, arm::stopClimber, arm));

        controller.button(4).whileTrue(new StartEndCommand(manipulator::grab, manipulator::stop, manipulator));
        controller.button(6).whileTrue(new StartEndCommand(manipulator::release, manipulator::stop, manipulator));
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
        // TODO: Implement properly
        return null;
    }
}