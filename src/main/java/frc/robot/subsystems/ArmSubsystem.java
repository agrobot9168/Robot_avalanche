package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.OperatorConstants;
import frc.robot.XboxControls;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ArmSubsystem extends SubsystemBase {
    
private final PIDController pidaArm;

  private final CommandXboxController gatilho =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

// motor

 private final TalonFX motorArm;

 // configuracao talon fx 

 private TalonFXConfiguration configArm;

 // Deadband

 public double DEADBAND = 0.08;

 // constantes
 public static boolean coleta = false;
 double velocityArm;
 private final double reducaoArm = 125.0;
 private final double correnteMAX = 40.0;

 // Limites mecânicos:
  private final double limiteMinGrausFisico = 0.2;
  private final double limiteMaxgrausFisico = 131.7;
 
  // controle de posicao 
  private final MotionMagicVoltage controlePosicao = new MotionMagicVoltage(0);
 
  // encoder 
  private final static DutyCycleEncoder encoderArm = new DutyCycleEncoder(9, 1, 0.9415790235394755);
  
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
  
  configArm.MotionMagic.MotionMagicCruiseVelocity = 3.0;
  configArm.MotionMagic.MotionMagicAcceleration = 1.0;
  configArm.MotionMagic.MotionMagicJerk = 0.01;
  
  // envia configuracao ao controlador
  
  motorArm.getConfigurator().apply(configArm);
  
  }
  
  // conversao de posicao 
  
  public static double getPosition_Arm() {
  
 
 
  double posicao = getEncoder_Arm();
  
  double graus = (posicao * 360.0);
  if(graus > 300){
    graus = 0.2;
  }

 
 return MathUtil.clamp(graus, 0.2, 140);
 
 }
     public static double getArmyEncoder() {
  
 
 
  double posicao = getEncoder_Arm();
  
  return (posicao * 360.0);
    }
 
 public static double getEncoder_Arm() {
     return encoderArm.get();
}

public double pidOutput() {
        gatilho.y().onTrue(Commands.runOnce(()-> coleta = true));
        gatilho.x().onTrue(Commands.runOnce(()-> coleta = false));
    if(coleta){
        return pidaArm.calculate(getPosition_Arm(), limiteMaxgrausFisico);
    }
    else{
        return pidaArm.calculate(getPosition_Arm(), limiteMinGrausFisico);
    }
           
}
 public void controlarArm() {
    double saidaPID = pidOutput();
    saidaPID = MathUtil.clamp(saidaPID, -0.6, 0.6); 
    SmartDashboard.putNumber("saidaPIDARMY", saidaPID);

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

// smartDashboard

SmartDashboard.putNumber("POSICAO ARM GRAUS", getPosition_Arm());

SmartDashboard.putNumber("ROTACOES MOTOR ARM", motorArm.getPosition().getValueAsDouble());

SmartDashboard.putNumber("CORRENTE ARM",  motorArm.getStatorCurrent().getValueAsDouble());

SmartDashboard.putNumber("ENCODER ARM",getEncoder_Arm());
SmartDashboard.putNumber("ARMY_ABS_ENCODER", getArmyEncoder());


SmartDashboard.putNumber("PID ARM", pidOutput());

    
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
