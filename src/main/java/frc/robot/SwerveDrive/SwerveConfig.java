package frc.robot.SwerveDrive;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;

public class SwerveConfig {
    // =========================
    // Pigeon2.0
    // =========================
    public final Pigeon2 pigeon2 = new Pigeon2(13);
    // =========================
    // Pigeon2.0
    // =========================

    // =========================
    // CAN IDs - COLOQUE OS SEUS
    // =========================

    public final CANcoder canFL = new CANcoder(10);
    public final CANcoder canFR = new CANcoder(11);
    public final CANcoder canBL = new CANcoder(9);
    public final CANcoder canBR = new CANcoder(12);

    // =========================
    // MOTORES DE ANGLE
    // =========================

    
    // 3 Minions = TalonFXS

    public final TalonFX motorBR = new TalonFX(4);
    public final TalonFX motorFR = new TalonFX(2);
    public final TalonFX motorBL = new TalonFX(6);
    public final TalonFX motorFL = new TalonFX(8);
    // public final TalonFXS motorIndex1 = new TalonFXS(19);

    // // Kraken X44

    

    // MOTORES DRIVE TALON FX

     public final TalonFX drive_motorBR = new TalonFX(3);
     public final TalonFX drive_motorFR = new TalonFX(1);
     public final TalonFX drive_motorBL = new TalonFX(5);
     public final TalonFX drive_motorFL = new TalonFX(7);

 
    //  MOTORS ANGLES SPARKMAX
    //public final SparkMax motorBR = new SparkMax(4, MotorType.kBrushless);
    //public final SparkMax motorFR = new SparkMax(3, MotorType.kBrushless);
    //public final SparkMax motorBL = new SparkMax(6, MotorType.kBrushless);
   // public final SparkMax motorFL = new SparkMax(5, MotorType.kBrushless);
    //  MOTORS ANGLES SPARKMAX

    //MOTORS DRIVE 
    //public final SparkMax drive_motorBR = new SparkMax(1, MotorType.kBrushless); 
    //public final SparkMax drive_motorFR = new SparkMax(2, MotorType.kBrushless);
    //public final SparkMax drive_motorBL = new SparkMax(8, MotorType.kBrushless);
    //public final SparkMax drive_motorFL = new SparkMax(7, MotorType.kBrushless);
    //MOTORS DRIVE

    // OFFSETS DOS CANCODERS
    public final double offsetFL = 276.41592;
    public final double offsetFR = 92.0214;
    public final double offsetBL = 108.72072;
    public final double offsetBR = 101.7774;
    

    //========================
    // CONTROLES
    // =========================

    // public final DutyCycleOut outputFL = new DutyCycleOut(0);
    // public final DutyCycleOut outputFR = new DutyCycleOut(0);
    // public final DutyCycleOut outputBL = new DutyCycleOut(0);
    // public final DutyCycleOut outputBR = new DutyCycleOut(0);

    // kP inicial
    public final PIDController pidFL = new PIDController(0.01, 0, 0);
    public final PIDController pidFR = new PIDController(0.01, 0, 0);
    public final PIDController pidBL = new PIDController(0.01, 0, 0);
    public final PIDController pidBR = new PIDController(0.01, 0, 0);
}
