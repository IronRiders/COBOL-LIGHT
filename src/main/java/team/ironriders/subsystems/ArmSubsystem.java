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

        pivotSpeedChooser.setDefaultOption("Default", String.valueOf(Constants.PIVOT_SPEED));
        for (double i = 0.2; i < 1; i += 0.1) {
            pivotSpeedChooser.addOption(String.format("Speed %.1f", i), String.valueOf(i));
        }

        climberSpeedChooser.setDefaultOption("Default", String.valueOf(Constants.CLIMBER_SPEED));
        for (double i = 0.2; i < 1; i += 0.1) {
            climberSpeedChooser.addOption(String.format("Speed %.1f", i), String.valueOf(i));
        }

        SmartDashboard.putData(pivotSpeedChooser);
        SmartDashboard.putData(climberSpeedChooser);
        SmartDashboard.putBoolean("Pivot", true);
        SmartDashboard.putBoolean("Climber", true);
    }

    @Override
    public void periodic() {
        pivotMultiplier = Double.parseDouble(pivotSpeedChooser.getSelected());
        climberMultiplier = Double.parseDouble(climberSpeedChooser.getSelected());

        SmartDashboard.putNumber("Pivot Rotation", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Climber Extension", climberMotor.getEncoder().getPosition() * -1);

        pivotMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 1.5f);
        // disables pivot in when climber is extended further than 35
        if (climberMotor.getEncoder().getPosition() * -1 > 35 && pivotMotor.getEncoder().getPosition() < 0.6f) {
            pivotMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 1.5f);
        } else {
            pivotMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0f);
        }
        pivotMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        pivotMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0f);
        // disables climber from extending out when pivot is lower than 0.5
        if (pivotMotor.getEncoder().getPosition() < 0.5) {
            climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0f);
        } else {
            climberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -220f);
        }
        climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        climberMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    }

    public void raise() {
        if (!SmartDashboard.getBoolean("Pivot", false)) { return; }
        pivotMotor.set(pivotMultiplier);
    }

    public void lower() {
        if (!SmartDashboard.getBoolean("Pivot", false)) { return; }
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
