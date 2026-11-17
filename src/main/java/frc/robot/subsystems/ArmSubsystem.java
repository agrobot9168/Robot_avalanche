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

 public static boolean L1 = false;
 public static boolean L2 = false;
 public static boolean L3 = false;

 double velocityArm;
 private final double reducaoArm = 125.0;
 private final double correnteMAX = 40.0;

 private final double l1Pose = 80;
 private final double l2Pose = 80;
 private final double l3Pose = 80;

 // Limites mecânicos:
  private final double limiteMinGrausFisico = 0.2;
  private final double limiteMaxgrausFisico = 140;
 
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
  configArm.MotionMagic.MotionMagicAcceleration = 0.5;
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

 
 return MathUtil.clamp(graus, 0.2, 150);
 
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
        
        gatilho.povDown().onTrue(Commands.runOnce(()-> L1 = true));
        gatilho.povLeft().onTrue(Commands.runOnce(()-> L2 = true));
        gatilho.povUp().onTrue(Commands.runOnce(()-> L3 = true));


    if(coleta && !L1 && !L2 && !L3){  //permite coleta somente quando L1, L2 e L3 forem falsos
        return pidaArm.calculate(getPosition_Arm(), limiteMaxgrausFisico);
    }
    else if(!coleta){
        L1 = false;
        L2 = false;
        L3 = false;
        return pidaArm.calculate(getPosition_Arm(), limiteMinGrausFisico);
    }

    if(L1){
        L2 = false;
        L3 = false;
        return pidaArm.calculate(getPosition_Arm(), l1Pose);
    } else if(L2) {
        L1 = false;
        L3 = false;
        return pidaArm.calculate(getPosition_Arm(), l2Pose);
    } else {
        L2 = false;
        L1 = false;
        return pidaArm.calculate(getPosition_Arm(), l3Pose);
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
