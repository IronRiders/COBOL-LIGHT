package team.ironriders.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.ironriders.constants.Constants;

public class MecanumWheelSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final PIDController pidController;
    private static final SimpleMotorFeedforward feedForward =
            new SimpleMotorFeedforward(0.148, 2.0004, 0.48);

    public MecanumWheelSubsystem(int motorId, boolean inverted) {
        motor = new CANSparkMax(motorId, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.setInverted(inverted);
        motor.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pidController = new PIDController(0.000003, 0, 0);
        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(Constants.WHEEL_CIRCUMFERENCE / Constants.GEARING);
    }

    public double getVelocity() {
        return (encoder.getVelocity() / Constants.GEARING / 60 * Constants.WHEEL_CIRCUMFERENCE);
    }

    public void setVelocity(double mps, boolean needPID) {
        if (needPID) {
            motor.setVoltage(feedForward.calculate(-mps) + pidController.calculate(getVelocity(), -mps));
        } else {
            motor.setVoltage(feedForward.calculate(mps));
        }
    }

    public static double getMaxLinearVelocity() {
        return feedForward.maxAchievableVelocity(12.0, 0);
    }

    public double getWheelPositions() {
        return encoder.getPosition();
    }

    public void burnFlash() {
        motor.burnFlash();
    }
}
