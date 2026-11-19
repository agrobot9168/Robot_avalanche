package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private boolean teste = false;

    private final SparkMax mElevator;
    private final SparkMaxConfig cfgElevator;
    private final SparkClosedLoopController pidElevator;

    private final RelativeEncoder elevatorEncoder;

    private double kP = 2;
    private double kD = 0.0;
    private double kI = 0.0;
    private double kV = 12 / 5676.0;

    private static final double maxVelocity = 5600.0;
    private static final double maxAceleration = 5600 * 2;
    private static final double aallowedPrrofileError = 0.1;

    public ElevatorSubsystem() {
        mElevator = new SparkMax(17, MotorType.kBrushless);
        cfgElevator = new SparkMaxConfig();
        pidElevator = mElevator.getClosedLoopController();
        elevatorEncoder = mElevator.getEncoder();

        mElevator.getEncoder().setPosition(0);

        cfgElevator.inverted(false).idleMode(IdleMode.kBrake);
        cfgElevator.smartCurrentLimit(30);
        cfgElevator.closedLoop.p(kP).d(kD).i(kI);
        cfgElevator.closedLoop.feedForward.kV(kV);
        cfgElevator.closedLoop.maxMotion
                                        .cruiseVelocity(maxVelocity)
                                        .maxAcceleration(maxAceleration)
                                        .allowedProfileError(aallowedPrrofileError);

        mElevator.configure(cfgElevator, com.revrobotics.ResetMode.kResetSafeParameters, 
                                            com.revrobotics.PersistMode.kNoPersistParameters);
    }

    public double getCurrent() {
        return mElevator.getOutputCurrent();
    }

    public double getPose_Elev() {
        return mElevator.getEncoder().getPosition();
    }

    public void setPose_Elev(double pose )  {
        pose = MathUtil.clamp(pose, -174, 0);
        pidElevator.setSetpoint(pose, ControlType.kMAXMotionPositionControl);
        
    }

    public void setVel_Elev(double vel) {
        mElevator.set(vel);
    }

    @Override
    public void periodic() {
        teste = !teste;
        SmartDashboard.putNumber("Elev_Pose", getPose_Elev());
        SmartDashboard.putBoolean("Elev_Vivo", teste);

        SmartDashboard.putNumber("Current_Elev", getCurrent());
    }

    public void setReference() {
        mElevator.getEncoder().setPosition(0);
    }

    public boolean CanMove() {
        return getPose_Elev() <= 0 && getPose_Elev() >= -174; 
    }

    public void freeze() {
        double freze = getPose_Elev();
        freze = MathUtil.clamp(freze, -174, 0);
        setPose_Elev(freze);
    }
}
