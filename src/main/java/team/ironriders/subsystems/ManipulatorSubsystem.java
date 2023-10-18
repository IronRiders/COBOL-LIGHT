package team.ironriders.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.ironriders.constants.Constants;

public class ManipulatorSubsystem extends SubsystemBase {
    private final CANSparkMax manipulatorRightMotor;
    private final CANSparkMax manipulatorLeftMotor;
    double speedMultiplier = Constants.MANIPULATOR_SPEED;

    public ManipulatorSubsystem() {
        manipulatorRightMotor = new CANSparkMax(Constants.MANIPULATOR_PORT1, CANSparkMaxLowLevel.MotorType.kBrushless);
        manipulatorLeftMotor = new CANSparkMax(Constants.MANIPULATOR_PORT2, CANSparkMaxLowLevel.MotorType.kBrushless);

        manipulatorRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        manipulatorRightMotor.setInverted(true);
        manipulatorRightMotor.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);

        manipulatorLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        manipulatorLeftMotor.setSmartCurrentLimit(Constants.MANIPULATOR_CURRENT_LIMIT);
    }

    @Override
    public void periodic() {
        manipulatorRightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);
        manipulatorRightMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -30);
        manipulatorLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 0);
        manipulatorLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -30);

        // This needs to be here so that if the robot starts up without redeploying the soft limits are enabled
        boolean enabled = false;
        manipulatorRightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, enabled);
        manipulatorRightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, enabled);
        manipulatorLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, enabled);
        manipulatorLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, enabled);

        // double rightPos = manipulatorRightMotor.getEncoder().getPosition() * -1;
        // double leftPos = manipulatorLeftMotor.getEncoder().getPosition() * -1;

        /*
        it is impossible to check for stalls from the current alone

        double rightCur = manipulatorRightMotor.getOutputCurrent();
        double leftCur = manipulatorLeftMotor.getOutputCurrent();

        double maxCur = 10;
        // If one of the manipulator motors stalls, and it is not being ignored
        if (!ignoreHighCurrent) {
            // Only stops the stalling manipulator and stops it
            if (rightCur > maxCur) {
                manipulatorLeftMotor.set(0);
            }
            if (leftCur > maxCur) {
                manipulatorLeftMotor.set(0);
            }
        }
         */

        // SmartDashboard.putNumber("Manipulator Right", rightPos);
        // SmartDashboard.putNumber("Manipulator Left", leftPos);
    }

    public void grab() {
        setManipulatorMotors(-1);
    }

    public void release() {
        setManipulatorMotors(1);
    }

    public void setManipulatorMotors(double speed) {
        manipulatorRightMotor.set(speed * 0.5 * speedMultiplier);
        manipulatorLeftMotor.set(speed * 0.5 * speedMultiplier);
    }

    public void stop() {
        setManipulatorMotors(0);
    }

    public double getRightPos() {
        return manipulatorRightMotor.getEncoder().getPosition();
    }

    public double getLeftPos() {
        return manipulatorLeftMotor.getEncoder().getPosition();
    }

    public void resetEncoders() {
        manipulatorRightMotor.getEncoder().setPosition(0);
        manipulatorLeftMotor.getEncoder().setPosition(0);
    }

    public void burnFlash() {
        manipulatorRightMotor.burnFlash();
        manipulatorLeftMotor.burnFlash();
    }
}

