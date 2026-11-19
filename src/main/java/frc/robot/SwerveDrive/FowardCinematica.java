
package frc.robot.SwerveDrive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FowardCinematica {

    private static double Vf, positionI, positionF, Vi = 0;

    private static final double GEAR_RATIO = 4.67;
    private static final double WHEEL_DIAMETER = 0.1016;

    private static final double L = 0.28128;
    private static final double W = 0.28128;


    private double positionX = 0;
    private double positionY = 0;
    private double positionTheta = 0;

    private double previousVx = 0;
    private double previousVy = 0;
    private double previousOmega = 0;

    private double previousTime;


    private static final double WHEEL_CIRCUMFERENCE =
        Math.PI * WHEEL_DIAMETER;

    private SwerveConfig swerveConfig = new SwerveConfig();

    public FowardCinematica() {
        previousTime = Timer.getFPGATimestamp();
    }

    // Velocidade linear de cada roda (m/s)
    public double getYaw (Pigeon2 pigeon) {
        return pigeon.getYaw().getValueAsDouble();
    }
    
    public double getMPS(TalonFX motor) {

        double motorRPS =
            motor.getRotorVelocity().getValueAsDouble();

        double wheelRPS = motorRPS / GEAR_RATIO;

        return wheelRPS * WHEEL_CIRCUMFERENCE;
    }
    //Orientação de cada rodinha
        private double getAngle(CANcoder cancoder, double offset) {

        double rotations =
                cancoder
                        .getAbsolutePosition()
                        .getValueAsDouble();

        double degrees =
                rotations * 360.0;

        degrees -= offset;

        return MathUtil.inputModulus(
                degrees,
                0,
                360
        );
    }
    double getOrientationFL() {
        return getAngle(swerveConfig.canFL, swerveConfig.offsetFL);
    }
    double getOrientationFR() {
        return getAngle(swerveConfig.canFR, swerveConfig.offsetFR);
    }
    double getOrientationBL() {
        return getAngle(swerveConfig.canBL, swerveConfig.offsetBL);
    }
    double getOrientationBR() {
        return getAngle(swerveConfig.canBR, swerveConfig.offsetBR);
    }

    // vel de cada modulo 
    double getVelFL() {
        return getMPS(swerveConfig.drive_motorFL);
    }

    double getVelFR() {
        return getMPS(swerveConfig.drive_motorFR);
    }

    double getVelBL() {
        return getMPS(swerveConfig.drive_motorBL);
    }

    double getVelBR() {
        return getMPS(swerveConfig.drive_motorBR);
    }


    // ========================================================
        // COMPONENTES DE VELOCIDADE DE CADA RODA
        // ========================================================
        private double[] fowardCinematica() {
        // conversao dps angulos para radianos
        double FLangle = Math.toRadians(getOrientationFL());
        double FRangle = Math.toRadians(getOrientationFR());
        double BLangle = Math.toRadians(getOrientationBL());
        double BRangle = Math.toRadians(getOrientationBR());


        // componentes de velocidade de cada roda 
        double flVx = getVelFL() * Math.cos(FLangle);
        double flVy = getVelFL() * Math.sin(FLangle);

        double frVx = getVelFR() * Math.cos(FRangle);
        double frVy = getVelFR() * Math.sin(FRangle);

        double blVx = getVelBL() * Math.cos(BLangle);
        double blVy = getVelBL() * Math.sin(BLangle);

        double brVx = getVelBR() * Math.cos(BRangle);
        double brVy = getVelBR() * Math.sin(BRangle);
        
        // VEL TRANSLACIONAL DO ROBO 
        double vx =
                (flVx + frVx + blVx + brVx) / 4.0;

        double vy =
                (flVy + frVy + blVy + brVy) / 4.0;

        // VEEL ANGULAR POR MINIMOS QUADRADROS 

        double omegaNumerator = (
                  (-W * flVx + L * flVy)
                + ( W * frVx + L * frVy)
                + (-W * blVx - L * blVy)
                + ( W * brVx - L * brVy));
        

        double omegaDenominator = 4.0 * (L * L + W * W);

        double omega = omegaNumerator / omegaDenominator;

        return new double[] {vx, vy, omega};
        }

    // integracao de posicao 

    private void updatePosition(
        double vxRobot,
        double vyRobot,
        double omega,
        double yawDegrees) {

        double currentTime = Timer.getFPGATimestamp();

        double dt = currentTime - previousTime;

        previousTime = currentTime;
 
 
        if (dt <= 0 || dt >= 0.1) {
            previousVx = vxRobot;
            previousVy = vyRobot;
            previousOmega = omega;

            return;
        }

        // convercao do yaw para radianos 
         
        double yaw = Math.toRadians(yawDegrees);

        // transformacao do referencial do robo para o campo

        double vxField = 
            vxRobot * Math.cos(yaw)
          - vyRobot * Math.sin(yaw);

        double vyField = 
         vxRobot * Math.sin(yaw)
        + vyRobot * Math.cos(yaw);

        // integracao trapezoidal independente 

        positionX += 
            ((previousVxField(vxField) + vxField) / 2.0) * dt;
        
        positionY += 
            ((previousVyField(vyField) + vyField) / 2.0) * dt;

        positionTheta += 
            ((previousOmega + omega ) / 2.0) * dt;

        // atualizacao das velocidades anteriores 

        previousVx = vxRobot;
        previousVy = vyRobot;
        previousOmega = omega;

        previousVxField =  vxField;
        previousVyField =  vyField;

    }
    
    // evita integrecao com intervalos invaldos ou muito grandes 

  
    public double GetPosition(double derivative) {
    Vf = derivative;

    positionI = ((Vi + Vf) / 2.0) * 0.02;
    positionF += positionI;

    Vi = Vf;

    return positionF;
    }

    // velocidades anteriores no referenfial do campo 

    private double previousVxField = 0;
    private double previousVyField = 0;

    private double previousVxField(double current) {
        return previousVxField;
    }

     private double previousVyField(double current) {
        return previousVyField;
    }
    

    public void Prints_FowardCinematicas() {
        double[] cinematica = fowardCinematica();

        double vx_final = cinematica[0];
        double vy_final = cinematica[1];
        double omega_final = cinematica[2];

        updatePosition(vx_final, vy_final, omega_final, getYaw(swerveConfig.pigeon2));


        SmartDashboard.putNumber("Swerve/VelocidadeDrive/FR", getVelFR());
        SmartDashboard.putNumber("Swerve/VelocidadeDrive/FL", getVelFL());
        SmartDashboard.putNumber("Swerve/VelocidadeDrive/BR", getVelBR());
        SmartDashboard.putNumber("Swerve/VelocidadeDrive/BL", getVelBL());

        SmartDashboard.putNumber("Swerve/AngulosDrive/FR", getOrientationFR());
        SmartDashboard.putNumber("Swerve/AngulosDrive/FL", getOrientationFL());
        SmartDashboard.putNumber("Swerve/AngulosDrive/BR", getOrientationBR());
        SmartDashboard.putNumber("Swerve/AngulosDrive/BL", getOrientationBL());

        // SmartDashboard.putNumber("Swerve/Testes/FlVx", flVx);
        // SmartDashboard.putNumber("Swerve/Testes/FlVy", flVy);

        SmartDashboard.putNumber("Swerve/Robo/Vx", vx_final);
        SmartDashboard.putNumber("Swerve/Robo/Vy", vy_final);
        SmartDashboard.putNumber("Swerve/Robo/Teta", omega_final);

        SmartDashboard.putNumber("RobotPosition/X", GetPosition(positionX));
        SmartDashboard.putNumber("RobotPosition/Y", GetPosition(positionY));
        SmartDashboard.putNumber("RobotPosition/Omega", GetPosition(positionTheta));
    }
}
