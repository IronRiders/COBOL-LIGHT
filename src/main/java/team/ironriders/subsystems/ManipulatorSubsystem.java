package team.ironriders.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import team.ironriders.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
    private final CANSparkMax manipulatorRightMotor;
    private final CANSparkMax manipulatorLeftMotor;
    SendableChooser<String> manipulatorSpeedChooser = new SendableChooser<>();
    double speedMultiplier = 0.1;

    public ManipulatorSubsystem() {
        manipulatorRightMotor = new CANSparkMax(Constants.MANIPULATOR_PORT1, CANSparkMaxLowLevel.MotorType.kBrushless);
        manipulatorLeftMotor = new CANSparkMax(Constants.MANIPULATOR_PORT2, CANSparkMaxLowLevel.MotorType.kBrushless);

        manipulatorRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        manipulatorRightMotor.setInverted(true);
        manipulatorRightMotor.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);

        manipulatorLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        manipulatorLeftMotor.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);

        manipulatorSpeedChooser.setDefaultOption("Speed 0.1", "0.1");
        for (double i = 0.2; i < 1; i += 0.1) {
            manipulatorSpeedChooser.addOption(String.format("Speed %.1f", i), String.valueOf(i));
        }

        SmartDashboard.putData(manipulatorSpeedChooser);
        SmartDashboard.putBoolean("Manipulator", false);
    }

    @Override
    public void periodic() {
        speedMultiplier = Double.parseDouble(manipulatorSpeedChooser.getSelected());
    }

    public void grab() {
        setManipulatorMotors(-speedMultiplier);
    }

    public void release() {
        setManipulatorMotors(speedMultiplier);
    }

    public void setManipulatorMotors(double speed) {
        if (!SmartDashboard.getBoolean("Manipulator", false)) { return; }

        manipulatorRightMotor.set(speed * 0.5);
        manipulatorLeftMotor.set(speed * 0.5);
    }

    public void stop() {
        setManipulatorMotors(0);
    }

    public void burnFlash() {
        manipulatorRightMotor.burnFlash();
        manipulatorLeftMotor.burnFlash();
    }
}

