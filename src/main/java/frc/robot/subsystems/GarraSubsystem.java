
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GarraSubsystem extends SubsystemBase {

    // =========================================================================================
    //                         CONFIGURAÇÃO   MOTOR   GARRA
    // =========================================================================================

    private final TalonFXS motorGarra;

    public GarraSubsystem() {

        motorGarra = new TalonFXS(14);

        TalonFXSConfiguration configs =
                new TalonFXSConfiguration();

        configs.Commutation.MotorArrangement =
                MotorArrangementValue.Minion_JST;

        motorGarra.getConfigurator().apply(configs);
    }

    // ========================================================================================
    //                               CONTROLE   DE  CORRENTE
    // ========================================================================================

    public double correnteAlvo = 10.0;

    public final PIDController pid_FXS =
            new PIDController(0.1, 0, 0);

    private final double SAIDA_MAXIMA = 0.15;

    private boolean controleCorrenteAtivo = false;

    // ========================================================================================
    //                                CONTROLE  DO  MOTOR
    // ========================================================================================

    public void setMotor(double velocidade) {

        controleCorrenteAtivo = false;

        motorGarra.set(
                MathUtil.clamp(velocidade, -1.0, 1.0)
        );
    }

    public void stopMotor() {

        controleCorrenteAtivo = false;

        motorGarra.set(0);
    }

    // =========================================================================================
    //                              LEITURA  DE  CORRENTE
    // =========================================================================================

    public double getCurrent() {

        return motorGarra
                .getStatorCurrent()
                .getValueAsDouble();
    }

    // =========================================================================================
    //                          CONTROLE  DE  TORQUE  ESTIMADO
    // =========================================================================================

    public void ativarControleCorrente() {

        pid_FXS.reset();

        controleCorrenteAtivo = true;
    }

    public void desativarControleCorrente() {

        controleCorrenteAtivo = false;

        motorGarra.set(0);
    }

    public double calcularCorrecaoCorrente() {

        double correnteAtual = getCurrent();

        return pid_FXS.calculate(
                correnteAtual,
                correnteAlvo
        );
    }

    public double calcularSaidaLimitada() {

        double correcao = calcularCorrecaoCorrente();

        return MathUtil.clamp(
                correcao,
                0.0,
                SAIDA_MAXIMA
        );
    }

    // ====================================================================================
    //                                    PERIODIC
    // ====================================================================================

    @Override
    public void periodic() {

        double corrente = getCurrent();

        double correcao = 0.0;

        if (controleCorrenteAtivo) {

            correcao = calcularSaidaLimitada();

            motorGarra.set(correcao);
        }

        SmartDashboard.putNumber(
                "CORRECAO",
                correcao
        );

        SmartDashboard.putNumber(
                "CORRENTE MOTOR FXS",
                corrente
        );

        SmartDashboard.putNumber(
                "CORRENTE ALVO",
                correnteAlvo
        );

        SmartDashboard.putBoolean(
                "CONTROLE CORRENTE ATIVO",
                controleCorrenteAtivo
        );
    }
}
