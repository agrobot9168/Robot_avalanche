
package frc.robot.SwerveDrive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FowardCinematica {

    private static double Vf, positionI, positionF, Vi = 0;

    private static final double GEAR_RATIO = 4.67;
    private static final double WHEEL_DIAMETER = 0.1016;

    private static final double WHEEL_CIRCUMFERENCE =
        Math.PI * WHEEL_DIAMETER;

    private SwerveConfig swerveConfig = new SwerveConfig();

    public FowardCinematica() {

    }

    // Velocidade linear de cada roda (m/s)
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
        private double[] fowardCinematica(){
        double flVx = getVelFL() * Math.cos(getOrientationFL());
        double flVy = getVelFL() * Math.sin(getOrientationFL());

        double frVx = getVelFR() * Math.cos(getOrientationFR());
        double frVy = getVelFR() * Math.sin(getOrientationFR());

        double blVx = getVelBL() * Math.cos(getOrientationBL());
        double blVy = getVelBL() * Math.sin(getOrientationBL());

        double brVx = getVelBR() * Math.cos(getOrientationBR());
        double brVy = getVelBR() * Math.sin(getOrientationBR());

        double vx =
                (flVx + frVx + blVx + brVx) / 4.0;

        double vy =
                (flVy + frVy + blVy + brVy) / 4.0;

        double omega = (
                  (-flVx + flVy)
                + ( frVx + frVy)
                + (-blVx - blVy)
                + ( brVx - brVy)
        ) / (4.0 * (0.28128 + 0.28128));

        return new double[] {vx, vy, omega};
        }

        

    public double GetPosition(double derivative) {
    Vf = derivative;

    positionI = ((Vi + Vf) / 2.0) * 0.02;
    positionF += positionI;

    Vi = Vf;

    return positionF;
    }

    public void Prints_FowardCinematicas() {
        double[] cinematica = fowardCinematica();

        double vx_final = cinematica[0];
        double vy_final = cinematica[1];
        double omega_final = cinematica[2];

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

        SmartDashboard.putNumber("RobotPosition/X", GetPosition(vx_final));
        SmartDashboard.putNumber("RobotPosition/Y", GetPosition(vy_final));
        SmartDashboard.putNumber("RobotPosition/Omega", GetPosition(omega_final));
    }
}
