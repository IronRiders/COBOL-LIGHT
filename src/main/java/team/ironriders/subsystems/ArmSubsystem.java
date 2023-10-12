package team.ironriders.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.ironriders.constants.Constants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax pivotMotor;
    private final CANSparkMax climberMotor;
    private final double pivotMultiplier = Constants.PIVOT_SPEED;
    private final double climberMultiplier = Constants.CLIMBER_SPEED;
    private boolean usingPIDPivot = false;
    private double pivotTarget = 0;
    private boolean usingPIDClimber = false;
    private double climberTarget = 0;

    private final ProfiledPIDController pivotPID = new ProfiledPIDController(
            Constants.Pivot_KP,
            0,
                    0,
                    new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(Constants.SHOULDER_VELOCITY_DEG),
                Units.degreesToRadians(Constants.SHOULDER_ACCELERATION_DEG)));

    private final PIDController climberPID = new PIDController(
            Constants.ARM_KP,
            0,
            0);

    public ArmSubsystem() {
        pivotMotor = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
        pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotMotor.setSmartCurrentLimit(Constants.PIVOT_CURRENT_LIMIT);

        climberMotor = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
        climberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        climberMotor.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT);
    }

    @Override
    public void periodic() {
        if (usingPIDPivot) {
            double result = pivotPID.calculate(pivotMotor.getEncoder().getPosition(), pivotTarget);
            pivotMotor.set(MathUtil.clamp(result, -1, 1) * (pivotMultiplier + 0.2));
        }
        if (usingPIDClimber) {
            double result = climberPID.calculate(climberMotor.getEncoder().getPosition(), -climberTarget);
            climberMotor.set(MathUtil.clamp(result, -1, 1) * climberMultiplier);
        }

        SmartDashboard.putNumber("Pivot Rotation", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Climber Extension", climberMotor.getEncoder().getPosition() * -1);

        pivotMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 1.7f);
        // disables pivot in when climber is extended further than 35
        if (climberMotor.getEncoder().getPosition() * -1 > 35 && pivotMotor.getEncoder().getPosition() < 0.75f) {
            pivotMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 1.7f);
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
        usingPIDPivot = false;
        pivotMotor.set(pivotMultiplier);
    }

    public void lower() {
        usingPIDPivot = false;
        pivotMotor.set(-pivotMultiplier);
    }

    public double getPivotPos() {
        return pivotMotor.getEncoder().getPosition();
    }

    public void setPivot(double target) {
        usingPIDPivot = true;
        pivotTarget = target;
    }

    public void stopPivot() {
        pivotMotor.set(0);
        usingPIDPivot = false;
    }

    public void retract() {
        usingPIDClimber = false;
        climberMotor.set(climberMultiplier);
    }

    public void extend() {
        usingPIDClimber = false;
        climberMotor.set(-climberMultiplier);
    }

    public void setClimber(double target) {
        usingPIDClimber = true;
        climberTarget = target;
    }

    public double getClimberPos() {
        return -climberMotor.getEncoder().getPosition();
    }

    public void stopClimber() {
        climberMotor.set(0);
        usingPIDClimber = false;
    }

    public void resetEncoders() {
        pivotMotor.getEncoder().setPosition(0);
        climberMotor.getEncoder().setPosition(0);
    }

    public void burnFlash() {
        pivotMotor.burnFlash();
    }
}
