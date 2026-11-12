package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{

    private final SparkMax mElevator;
    private final SparkMaxConfig cfgElevator;
    private final SparkClosedLoopController pidElevator;

    private final RelativeEncoder elevatorEncoder;

    private double kP = 0.01;
    private double kD = 0.0;
    private double kI = 0.0;
    private double kV = 12 / 5676.0;

    private static final double maxVelocity = 5000.0;
    private static final double maxAceleration = 5600 * 2;
    private static final double aallowedPrrofileError = 0.1;

    public ElevatorSubsystem() {
        mElevator = new SparkMax(17, MotorType.kBrushless);
        cfgElevator = new SparkMaxConfig();
        pidElevator = mElevator.getClosedLoopController();
        elevatorEncoder = mElevator.getEncoder();

        elevatorEncoder.setPosition(0);

        cfgElevator.inverted(false).idleMode(IdleMode.kBrake);
        cfgElevator.smartCurrentLimit(20);
        cfgElevator.closedLoop.p(kP).d(kD).i(kI);
        cfgElevator.closedLoop.feedForward.kV(kV);
        cfgElevator.closedLoop.maxMotion
                                        .cruiseVelocity(maxVelocity)
                                        .maxAcceleration(maxAceleration)
                                        .allowedProfileError(aallowedPrrofileError);

        mElevator.configure(cfgElevator, com.revrobotics.ResetMode.kResetSafeParameters, 
                                            com.revrobotics.PersistMode.kNoPersistParameters);
    }

    public double getPose_Elev() {
        return elevatorEncoder.getPosition();
    }

    public void setPose_Elev(double pose) {
        pidElevator.setSetpoint(pose, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elev_Pose", getPose_Elev());
    }

    public void setReference() {
        elevatorEncoder.setPosition(0);
    }
}
