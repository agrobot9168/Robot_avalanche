
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class GarraSubsystem extends SubsystemBase {

    // =========================================================================================
    //                         CONFIGURAÇÃO   MOTOR   GARRA
    // =========================================================================================


    //                                    LOGICIA ATUAL
    //==========================================================================================
    //                                  Corrente >= 10 A
    //                          Registrar posição atual do encoder
    //                             Marcar referência definida
    //                  Ativar controle de posição para manter a referência
    //===========================================================================================


    // guardar a posição do motor
    private double posicaoSegurar = 0.0;

    private final TalonFXS motorGarra;

    // referencia corrente no travamento
    private double correnteReferencia = 25.0;


    // cria o controle com posição inicial igual a zero
    private final PositionVoltage controlePosicao = new PositionVoltage(0);


    // indica se o controle de posição deve estar ativo.
    private boolean mantendoPosicao = false;

    // incremento de correcao
    private double incrementoPosicao = -4;

    private boolean referenciaaDefinida = false;



    final PIDController controleGarra = new PIDController(0.01, 0, 0);


    public GarraSubsystem() {

        motorGarra = new TalonFXS(14);

        // TalonFXSConfiguration configs =
                // new TalonFXSConfiguration();

        // configs.Commutation.MotorArrangement =
                // MotorArrangementValue.Minion_JST;

        // configs.Slot0.kP = 5;
        // configs.Slot0.kI = 0.0;
        // configs.Slot0.kD = 0.0;

        // // motorGarra.getConfigurator().apply(configs);

       
    }

    // ========================================================================================
    //                                CONTROLE  DO  MOTOR
    // ========================================================================================

    public void setMotor(double velocidade) {
        
        motorGarra.set(MathUtil.clamp(velocidade, -1.0, 1.0));
        referenciaaDefinida = false;
        
    }

    public void stopMotor() {
        mantendoPosicao = false;
        
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
    public double erro;
    // ====================================================================================
    //                                    PERIODIC
    // ====================================================================================

    @Override
        public void periodic() {

    double corrente = getCurrent();

        // Se a corrente ultrapassar a desejada
        // Registrar posição inicial
        if (corrente >= correnteReferencia) {
                erro = controleGarra.calculate(
                getPosition(), (getPosition() + 2/360.0) 
                
            ) * -1 + getPosition();

             motorGarra.setControl(  
         controlePosicao.withPosition( erro));

//         posicaoSegurar = getPosition();

//         referenciaaDefinida = true;
//         mantendoPosicao = true;
}


        // Correção de posição
        // && corrente > correnteReferencia
//         if (referenciaaDefinida) {

//         posicaoSegurar += incrementoPosicao;
//         referenciaaDefinida = false; 
// }


        // Manter a posição desejada
//         if (mantendoPosicao) {
//            double  graus = 5;
//             double rotacao = graus / 360;
//         motorGarra.setControl(  
//         controlePosicao.withPosition(rotacao)
//     );
// }

    SmartDashboard.putNumber("CORRENTE MOTOR FXS", corrente);
    SmartDashboard.putNumber("POSICAO GARRA", getPosition());
    SmartDashboard.putNumber("POSICAO PARA SEGURAR", posicaoSegurar);
    SmartDashboard.putNumber("CORRENTE REFERENCIA", correnteReferencia);
    SmartDashboard.putBoolean("MANTENDO POSICAO", mantendoPosicao);
    SmartDashboard.putBoolean("REFERENCIADEFINIDA", referenciaaDefinida);
    SmartDashboard.putNumber("ERRO", erro);
}

     

    
    //=======================================================
    //                   PEGAR POSIÇÃO
    //=======================================================

    public double getPosition() {
           
        return motorGarra.getPosition().getValueAsDouble();
        
    }

}

