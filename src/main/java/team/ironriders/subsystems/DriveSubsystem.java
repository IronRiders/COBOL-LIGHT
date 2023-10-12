package team.ironriders.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team.ironriders.constants.Constants;

public class DriveSubsystem extends SubsystemBase {
    private boolean inverted;
    private ChassisSpeeds targetChassisSpeeds;
    private final MecanumWheelSubsystem frontLeftMotor;
    private final MecanumWheelSubsystem frontRightMotor;
    private final MecanumWheelSubsystem rearRightMotor;
    private final MecanumWheelSubsystem rearLeftMotor;
    private final MecanumDriveKinematics kinematics;
    public final WPI_Pigeon2 pigeon;
    public final Field2d field;
    private final MecanumDrivePoseEstimator poseEstimator;
    private static final ProfiledPIDController profiledThetaController =
            new ProfiledPIDController(
                    0.4,
                    0,
                    0,
                    new TrapezoidProfile.Constraints(
                            Units.rotationsToRadians(0.75), Units.rotationsToRadians(1.5)));
    private static final PIDController thetaController =
            new PIDController(
                    profiledThetaController.getP(),
                    profiledThetaController.getI(),
                    profiledThetaController.getD());
    private static final PIDController xController = new PIDController(0.15, 0, 0);
    private static final PIDController yController = new PIDController(0.15, 0, 0);

    public DriveSubsystem() {
        frontLeftMotor = new MecanumWheelSubsystem(Constants.WHEEL_PORT_FRONT_LEFT, true);
        frontRightMotor = new MecanumWheelSubsystem(Constants.WHEEL_PORT_FRONT_RIGHT, false);
        rearRightMotor = new MecanumWheelSubsystem(Constants.WHEEL_PORT_REAR_LEFT, true);
        rearLeftMotor = new MecanumWheelSubsystem(Constants.WHEEL_PORT_REAR_RIGHT, false);
        inverted = false;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // meter per second
        kinematics =
                new MecanumDriveKinematics(
                        new Translation2d(0.2413, 0.28893008),
                        new Translation2d(0.2413, -0.28893008),
                        new Translation2d(-0.2413, 0.28893008),
                        new Translation2d(-0.2413, -0.28893008));

        pigeon = new WPI_Pigeon2(15);
        targetChassisSpeeds = new ChassisSpeeds();
        field = new Field2d();
        poseEstimator =
                new MecanumDrivePoseEstimator(
                        getKinematics(), pigeon.getRotation2d(), getWheelPositions(), new Pose2d());
    }

    public void invertDrive() {
        inverted = !inverted;
    }

    public MecanumDriveWheelPositions getWheelPositions() {
        return new MecanumDriveWheelPositions(
                frontLeftMotor.getWheelPositions(),
                frontRightMotor.getWheelPositions(),
                rearLeftMotor.getWheelPositions(),
                rearRightMotor.getWheelPositions());
    }

    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        return new MecanumDriveWheelSpeeds(
                frontLeftMotor.getVelocity(),
                frontRightMotor.getVelocity(),
                rearRightMotor.getVelocity(),
                rearLeftMotor.getVelocity());
    }

    public void periodic() {
        // Simple Simulation
        field.setRobotPose(getPose2d());
        field
                .getObject("Target")
                .setPose(
                        new Pose2d(
                                xController.getSetpoint(),
                                yController.getSetpoint(),
                                new Rotation2d(getThetaController().getSetpoint())));

        SmartDashboard.putNumber("Gyro", Math.abs(pigeon.getAngle() % 360));

        // Tuning
        // NetworkTableInstance.getDefault().flush();

        /* SmartDashboard.putNumber("x controller", getPose2d().getX());
        SmartDashboard.putNumber("x Controller (target)", xController.getSetpoint());
        SmartDashboard.putNumber("Y controller", getPose2d().getY());
        SmartDashboard.putNumber("y Controller (target)", yController.getSetpoint());
        SmartDashboard.putNumber("Theta controller (Degrees)", getPose2d().getRotation().getDegrees());
        SmartDashboard.putNumber(
                "Theta setPoint (Target))", Math.toDegrees(profiledThetaController.getSetpoint().position));

        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getWheelSpeeds());

        SmartDashboard.putNumber("Drive/VX", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/VY", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber(
                "Drive/Omega Degrees", Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond));

        SmartDashboard.putNumber("Drive/Target VX", targetChassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drive/Target VY", targetChassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber(
                "Drive/Target Omega Degrees",
                Units.radiansToDegrees(targetChassisSpeeds.omegaRadiansPerSecond));
        SmartDashboard.putNumber("Gyro Angle", pigeon.getAngle());

        SmartDashboard.putNumber("Pitch", pigeon.getPitch());

         */
    }

    public Pose2d getPose2d() {
        return poseEstimator.getEstimatedPosition();
    }

    public MecanumDriveKinematics getKinematics() {
        return kinematics;
    }

    public void setWheelSpeeds(MecanumDriveWheelSpeeds speed, boolean needPID) {
        frontLeftMotor.setVelocity(speed.frontLeftMetersPerSecond, needPID);
        frontRightMotor.setVelocity(speed.frontRightMetersPerSecond, needPID);
        rearRightMotor.setVelocity(speed.rearLeftMetersPerSecond, needPID);
        rearLeftMotor.setVelocity(speed.rearRightMetersPerSecond, needPID);
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean needPID) {
        targetChassisSpeeds = chassisSpeeds;
        setWheelSpeeds(kinematics.toWheelSpeeds(targetChassisSpeeds), needPID);
    }

    public void setChassisSpeeds(double strafe, double drive, double turn, boolean needPID) {

        double xSpeed = drive * MecanumWheelSubsystem.getMaxLinearVelocity();
        double ySpeed = strafe * MecanumWheelSubsystem.getMaxLinearVelocity();
        double turnSpeed = turn * -getMaxRotationalVelocity();

        ChassisSpeeds chassisSpeeds;

        if (!needPID)
            chassisSpeeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, pigeon.getRotation2d());
        else chassisSpeeds = new ChassisSpeeds(-xSpeed, -ySpeed, -turnSpeed);

        setChassisSpeeds(chassisSpeeds, needPID);
    }

    public double getMaxRotationalVelocity() {
        return Math.abs(
                (kinematics.toChassisSpeeds(
                        new MecanumDriveWheelSpeeds(
                                MecanumWheelSubsystem.getMaxLinearVelocity(),
                                -MecanumWheelSubsystem.getMaxLinearVelocity(),
                                MecanumWheelSubsystem.getMaxLinearVelocity(),
                                -MecanumWheelSubsystem.getMaxLinearVelocity()
                        )
                )).omegaRadiansPerSecond
        );
    }

    public void stop() {
        setChassisSpeeds(0, 0, 0, true);
    }

    public void resetOdometry(Pose2d pose2d) {
        poseEstimator.resetPosition(pigeon.getRotation2d(), getWheelPositions(), pose2d);
    }

    public void resetPigeon() {
        pigeon.setYaw(0.0);
    }

    public MecanumDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public ProfiledPIDController getProfiledThetaController() {
        return profiledThetaController;
    }

    public PIDController getThetaController() {
        return thetaController;
    }

    public PIDController getxController() {
        return xController;
    }

    public PIDController getyController() {
        return yController;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    public void burnFlash() {
        frontLeftMotor.burnFlash();
        frontRightMotor.burnFlash();
        rearRightMotor.burnFlash();
        rearLeftMotor.burnFlash();
    }
}
