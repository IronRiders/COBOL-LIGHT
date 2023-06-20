package team.ironriders.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.ironriders.Constants;

public class ArmSubsystem extends SubsystemBase {
    SendableChooser<String> pivotSpeedChooser = new SendableChooser<>();
    SendableChooser<String> climberSpeedChooser = new SendableChooser<>();

    private final CANSparkMax pivotMotor;
    private final CANSparkMax climberMotor;
    private double pivotMultiplier = 0.1;
    private double climberMultiplier = 0.1;

    public ArmSubsystem() {
        pivotMotor = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
        pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotMotor.setSmartCurrentLimit(Constants.PIVOT_CURRENT_LIMIT);

        climberMotor = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
        climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        climberMotor.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT);

        pivotSpeedChooser.setDefaultOption("Speed 0.1", "0.1");
        for (double i = 0.2; i < 1; i += 0.1) {
            pivotSpeedChooser.addOption(String.format("Speed %.1f", i), String.valueOf(i));
        }

        climberSpeedChooser.setDefaultOption("Speed 0.1", "0.1");
        for (double i = 0.2; i < 1; i += 0.1) {
            climberSpeedChooser.addOption(String.format("Speed %.1f", i), String.valueOf(i));
        }

        SmartDashboard.putData(pivotSpeedChooser);
        SmartDashboard.putData(climberSpeedChooser);
        SmartDashboard.putBoolean("Pivot", false);
        SmartDashboard.putBoolean("Climber", false);
    }

    @Override
    public void periodic() {
        pivotMultiplier = Double.parseDouble(pivotSpeedChooser.getSelected());
        climberMultiplier = Double.parseDouble(climberSpeedChooser.getSelected());

        SmartDashboard.putNumber("Pivot Rotation", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Climber Rotation", pivotMotor.getEncoder().getPosition());
    }

    public void raise() {
        if (pivotMotor.getEncoder().getPosition() > 1.5 || !SmartDashboard.getBoolean("Pivot", false)) { return; }
        pivotMotor.set(pivotMultiplier);
    }

    public void lower() {
        if (pivotMotor.getEncoder().getPosition() < 0.35 || !SmartDashboard.getBoolean("Pivot", false)) { return; }
        pivotMotor.set(-pivotMultiplier);
    }

    public void retract() {
        if (!SmartDashboard.getBoolean("Climber", false)) { return; }
        climberMotor.set(climberMultiplier);
    }

    public void extend() {
        if (!SmartDashboard.getBoolean("Climber", false)) { return; }
        climberMotor.set(-climberMultiplier);
    }

    public void stopPivot() {
        pivotMotor.set(0);
    }

    public void stopClimber() {
        climberMotor.set(0);
    }

    public void resetEncoders() {
        pivotMotor.getEncoder().setPosition(0);
        climberMotor.getEncoder().setPosition(0);
    }

    public void burnFlash() {
        pivotMotor.burnFlash();
    }
}
