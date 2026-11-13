package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.XboxControls;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    
private final PIDController pidaArm;

// motor

 private final TalonFX motorArm;

 // configuracao talon fx 

 private TalonFXConfiguration configArm;

 // Deadband

 public double DEADBAND = 0.08;
 

 // constantes

 private final double reducaoArm = 125.0;

 private final double correnteMAX = 40.0;

 // Limites mecânicos:
 private final double limiteMinGrausFisico = 156.3;

 private final double limiteMaxgrausFisico = 294.7;

 // controle de posicao 

 private final MotionMagicVoltage controlePosicao = new MotionMagicVoltage(0);

 // encoder 
 private final static DutyCycleEncoder encoderArm = new DutyCycleEncoder(9, 1, 0);
 
 // construtor
 
 public ArmSubsystem() {
 
     pidaArm = new PIDController(0.01, 0.0, 0.001);
 
     // pidaArm.setTolerance(1.0); 
 
     motorArm = new TalonFX(15);
 
     configArm = new TalonFXConfiguration();
 
 // limite corrente 
     configArm.CurrentLimits.StatorCurrentLimit = correnteMAX;
     configArm.CurrentLimits.StatorCurrentLimitEnable = true;
 
 
 // modo brake
     configArm.MotorOutput.NeutralMode = NeutralModeValue.Brake;
 
 
 // Pid interno 
 
 configArm.Slot0.kP = 0.5;
 configArm.Slot0.kI = 0.0;
 configArm.Slot0.kD = 0.0;
 
 // motion Magic
 
 configArm.MotionMagic.MotionMagicCruiseVelocity = 5.0;
 configArm.MotionMagic.MotionMagicAcceleration = 10.0;
 
 // envia configuracao ao controlador
 
 motorArm.getConfigurator().apply(configArm);
 
 }
 
 // conversao de posicao 
 
 public static double getPosition_Arm() {
 
 double posicao = getEncoder_Arm();
 
 double graus = posicao * 360.0;
 
 return graus;
 
 }
 
 public static double getEncoder_Arm() {
     return encoderArm.get();
}

public double pidOutput() {
     return pidaArm.calculate(getPosition_Arm(), map(XboxControls.Get_LeftTriggerAxis(), 
                                                        0,
                                                        1,
                                                        limiteMinGrausFisico + 2,
                                                        limiteMaxgrausFisico - 2));
}
 public void controlarArm() {
    double saidaPID = pidOutput();
    saidaPID = MathUtil.clamp(saidaPID, -0.6, 0.6);

    motorArm.set(saidaPID);
}





// movimentacao 

// public void setPositionArm(double graus) {
    
//     double referencia = MathUtil.clamp(graus, limiteMinGrausFisico, limiteMaxgrausFisico);

//     double rotacoes = grausParaRotacoes(referencia);

//     motorArm.setControl(
//         controlePosicao.withPosition(rotacoes));
// }



//periodic

@Override

public void periodic() {

controlarArm();

// botao


// nunca mudar para public 
double gatilho = XboxControls.Get_LeftTriggerAxis();

// smartDashboard

SmartDashboard.putNumber("POSICAO ARM GRAUS", getPosition_Arm());

SmartDashboard.putNumber("ROTACOES MOTOR ARM", motorArm.getPosition().getValueAsDouble());

SmartDashboard.putNumber("CORRENTE ARM",  motorArm.getStatorCurrent().getValueAsDouble());

SmartDashboard.putNumber("ENCODER ARM",getEncoder_Arm());

SmartDashboard.putNumber("GATILHO ARM", gatilho);

SmartDashboard.putNumber("PID ARM", pidOutput());

SmartDashboard.putNumber("KKKKKK", map(XboxControls.Get_LeftTriggerAxis(), 
                                                        0,
                                                        1,
                                                        limiteMinGrausFisico,
                                                        limiteMaxgrausFisico));

    
    }

    public double map(
    double valor,
    double entradaMin,
    double entradaMax,
    double saidaMin,
    double saidaMax
) {

    return saidaMin +
        (valor - entradaMin) *
        (saidaMax - saidaMin) /
        (entradaMax - entradaMin);
}

}
