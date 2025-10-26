package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;


public class ArmSubsystem extends SubsystemBase{
    
    private TalonFX arm;
    private CANcoder armCANcoder;

    private static ArmSubsystem system;

    /**
     * Constructs an {@link ArmSubsystem} subsystem instance
     */
    private ArmSubsystem() {

        arm = new TalonFX(41);
        arm.getConfigurator().apply(armConfiguration());
        arm.setNeutralMode(NeutralModeValue.Brake);

        armCANcoder = new CANcoder(ArmConstants.CanID);
        armCANcoder.getConfigurator().apply(armCANCoderConfiguration());

    }

    public Command moveTo(double desiredPosition) {
        double Setpoint = desiredPosition / 360; // converts degrees to rotations
        return run(() -> {
            moveArm(Setpoint);
        });
    }

    /**
     * Runs just the arm to the supplied setpoint
     *
     * @param setpoint The target setpoint in rotations
     */

    public void moveArm(double setpoint) {
        arm.setControl(new PositionDutyCycle(setpoint));
        SmartDashboard.putNumber("Arm Setpoint", setpoint);
    }

    /**
     * Gets the Arm motor's encoder position
     *
     * @return The Arm motor encoder position in rotations
     */
    public double getArmPosition() {
        return arm.getPosition().getValueAsDouble();
    }

    /**
     * Stops the Arm motor
     */
    public void stopArm() {
        arm.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm pos", getArmPosition());
    }

    /**
     * Gets the {@link TalonFXConfiguration} for the Arm joint
     *
     * @return The {@link TalonFXConfiguration} for the Arm joint
     */
    private TalonFXConfiguration armConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.Feedback.SensorToMechanismRatio = 1.0;
        configuration.Feedback.RotorToSensorRatio = ArmConstants. CancoderGearRatio;

        configuration.Slot0 = new Slot0Configs()
            .withKS(ArmConstants.kS)
            .withKV(ArmConstants.kV)
            .withKA(ArmConstants.kA)
            .withKG(ArmConstants.kG)
            .withKP(ArmConstants.kP)
            .withKD(ArmConstants.kD);

        configuration.Slot0.GravityType = ArmConstants.GravityType;

        configuration.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ArmConstants.CruiseVelocity)
            .withMotionMagicAcceleration(ArmConstants.Acceleration)
            .withMotionMagicJerk(ArmConstants.Jerk);
        configuration.CurrentLimits.StatorCurrentLimit = ArmConstants.StatorCurrentLimit;
        configuration.CurrentLimits.SupplyCurrentLimit = ArmConstants.SupplyCurrentLimit;
        configuration.CurrentLimits.SupplyCurrentLowerTime = ArmConstants.SupplyCurrentLowerTime;

        configuration.Feedback.FeedbackRemoteSensorID = ArmConstants.CoderID;
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        return configuration;
    }

    /**
     * Gets the {@link CANcoderConfiguration} for the arm CANCoder
     *
     * @return The {@link CANcoderConfiguration} for the arm CANCoder
     */
    private CANcoderConfiguration armCANCoderConfiguration() {
        CANcoderConfiguration configuration = new CANcoderConfiguration();

        configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ArmConstants.CoderDiscontinuityPoint;
        configuration.MagnetSensor.MagnetOffset = ArmConstants.CoderOffset;
        configuration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        return configuration;
    }

    /**
     * Gets the {@link ArmSubsystem} subsystem instance
     *
     * @return The {@link ArmSubsystem} subsystem instance
     */
    public static ArmSubsystem system() {
        if (system == null) {
            system = new ArmSubsystem();
        }

        return system;
    }
}

