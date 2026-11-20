
package frc.robot.SwerveDrive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class DriveMotorController {

    // ========================================================
    // MOTOR
    // ========================================================

    private final TalonFX motor;

    // ========================================================
    // CONFIGURAÇÕES FÍSICAS
    // ========================================================
    private static final double GEAR_RATIO = 4.67;
    private static final double WHEEL_DIAMETER = 0.1016;
    private static final double WHEEL_CIRCUMFERENCE =
        Math.PI * WHEEL_DIAMETER;

    private final double MAX_VEL_MPS;
    private final double MAX_ACCEL_MPS2;
    private final double MAX_JERK_MPS3;

    // ========================================================
    // GANHOS PID
    // ========================================================

    private final double kP;
    private final double kI;
    private final double kD;
    private final double kS;
    private final double kV;

    // ========================================================
    // LIMITES DE CORRENTE
    // ========================================================

    private final double STATOR_CURRENT_LIMIT;
    private final double SUPPLY_CURRENT_LIMIT;

    // ========================================================
    // CONTROLE
    // ========================================================

    private final MotionMagicVelocityVoltage velocityRequest =
            new MotionMagicVelocityVoltage(0);

    // ========================================================
    // CONSTRUTOR
    // ========================================================

    public DriveMotorController(
            TalonFX motor,
            boolean inverted,

            double maxVelocityMPS,
            double maxAccelerationMPS2,
            double maxJerkMPS3,

            double kP,
            double kI,
            double kD,
            double kS,
            double kV,

            double statorCurrentLimit,
            double supplyCurrentLimit) {

        this.motor = motor;

        this.MAX_VEL_MPS = maxVelocityMPS;
        this.MAX_ACCEL_MPS2 = maxAccelerationMPS2;
        this.MAX_JERK_MPS3 = maxJerkMPS3;

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kS = kS;
        this.kV = kV;

        this.STATOR_CURRENT_LIMIT = statorCurrentLimit;
        this.SUPPLY_CURRENT_LIMIT = supplyCurrentLimit;

        configureMotor(inverted);
    }

    // ========================================================
    // CONFIGURAÇÃO DO TALONFX
    // ========================================================

    private void configureMotor(boolean inverted) {

        TalonFXConfiguration config =
                new TalonFXConfiguration();

        // Ganhos PID

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;

        
        // Motion Magic

        config.MotionMagic.MotionMagicAcceleration =
                mpsToMotorRPS(MAX_ACCEL_MPS2);

        config.MotionMagic.MotionMagicJerk =
                mpsToMotorRPS(MAX_JERK_MPS3);

        // Inversão

        config.MotorOutput.Inverted =
                inverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        // Modo de parada

        config.MotorOutput.NeutralMode =
                NeutralModeValue.Brake;

        // Limite de corrente

        config.CurrentLimits.StatorCurrentLimit =
                STATOR_CURRENT_LIMIT;

        config.CurrentLimits.StatorCurrentLimitEnable =
                STATOR_CURRENT_LIMIT > 0;

        config.CurrentLimits.SupplyCurrentLimit =
                SUPPLY_CURRENT_LIMIT;

        config.CurrentLimits.SupplyCurrentLimitEnable =
                SUPPLY_CURRENT_LIMIT > 0;

        // Aplicar configurações

        var status = motor.getConfigurator().apply(config);

        if (!status.isOK()) {
            System.out.println(
                    "ERRO AO CONFIGURAR TALONFX ID "
                    + motor.getDeviceID()
                    + ": " + status
            );
        }
    }

    // ========================================================
    // CONVERSÕES
    // ========================================================

    // m/s para RPS do rotor
    private double mpsToMotorRPS(double mps) {

        double wheelRPS =
                mps / WHEEL_CIRCUMFERENCE;

        return wheelRPS * GEAR_RATIO;
    }

    // RPS do rotor para m/s
    private double motorRPSToMPS(double motorRPS) {

        double wheelRPS =
                motorRPS / GEAR_RATIO;

        return wheelRPS * WHEEL_CIRCUMFERENCE;
    }

    // ========================================================
    // SET VELOCITY
    // ========================================================

    public void setVel(double velocityMPS) {

        // Limitar velocidade máxima

        double limitedVelocity = Math.max(
                -MAX_VEL_MPS,
                Math.min(MAX_VEL_MPS, velocityMPS)
        );

        // Converter para RPS do rotor

        double targetRPS =
                mpsToMotorRPS(limitedVelocity);

        // Aplicar Motion Magic Velocity

        motor.setControl(
                velocityRequest.withVelocity(targetRPS)
        );
    }

    // ========================================================
    // GET VELOCITY
    // ========================================================

    public double getVel() {

        double motorRPS =
                motor.getRotorVelocity().getValueAsDouble();

        return motorRPSToMPS(motorRPS);
    }

    // ========================================================
    // PARADA
    // ========================================================

    // Parada respeitando o perfil de desaceleração
    public void stop() {
        setVel(0);
    }

    // Parada imediata, sem seguir o perfil de velocidade
    public void stopImmediate() {
        motor.stopMotor();
    }

    // ========================================================
    // ACESSO AO MOTOR
    // ========================================================

    public TalonFX getMotor() {
        return motor;
    }

    public double getMaxVel() {
        return MAX_VEL_MPS;
    }
}
