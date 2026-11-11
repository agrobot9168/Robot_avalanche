
package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.XboxControls;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class HandSubsystem extends SubsystemBase {

    // motor
    private final TalonFXS hand;

    // configTalonFX
    private TalonFXSConfiguration configHand;

    private final XboxControls controls = new XboxControls();

    public double DEADBAND = 0.08;

    private final MotionMagicVoltage controlePosicao = new MotionMagicVoltage(0);

    public HandSubsystem() {

        hand = new TalonFXS(20);

        configHand = new TalonFXSConfiguration();

        configHand.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        configHand.CurrentLimits.StatorCurrentLimit = 40;
        configHand.CurrentLimits.StatorCurrentLimitEnable = true;

        configHand.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Ganhos PID internos do TalonFXS
        configHand.Slot0.kP = 0.5;
        configHand.Slot0.kI = 0;
        configHand.Slot0.kD = 0;
        configHand.MotionMagic.MotionMagicCruiseVelocity = 20.0;
        configHand.MotionMagic.MotionMagicAcceleration = 90.0;

        hand.getConfigurator().apply(configHand);

        hand.setPosition(0);
    }

    // travar posicao
    public void holdPosition() {

        double posicaoAtual = hand.getPosition().getValueAsDouble();

        hand.setControl(new PositionVoltage(posicaoAtual));
    }

    public double getPosition_Hand() {

        // le posicao do encoder em rotacoes
        double rotacoes = hand.getPosition().getValueAsDouble();

        // Converte a posição do motor para graus da articulação
        double graus = (rotacoes * 360.0) / 16.0;

        return graus;
    }

    // Movimentacao
    public void setSpeed(double velocidade) {
        hand.set(velocidade);
    }

    @Override
    public void periodic() {

        double voltas = 16.0;

        double gatilho = XboxControls.Get_RightTriggerAxis();

        double referencia;

        if (gatilho > DEADBAND) {

             referencia = gatilho * voltas;
        } else {

            referencia = 0;
        }
            hand.setControl(
                controlePosicao.withPosition(referencia)
            );

        SmartDashboard.putNumber("POSICAO DA HAND", getPosition_Hand());

        SmartDashboard.putNumber("GATILHO", gatilho);

        SmartDashboard.putNumber("ROTACOES MOTOR HAND",hand.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("REFERENCIA HAND",referencia);
    }
}
