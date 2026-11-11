// package frc.robot.SwerveDrive;

// import frc.robot.LimelightHelpers;

// import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

// import edu.wpi.first.math.MathUtil;

// public class AprilTagYaw {
    

//     private final String limelightName;

//     // Ganho proporcional
//     private double kP = 0.03;

//     // Máxima velocidade de rotação
//     private double maxOmega = 0.5;

//     // Considera alinhado quando TX estiver dentro disso
//     private double tolerance = 1.0;

//     // Inverte a direção caso necessário
//     private boolean inverted = false;

//     public AprilTagYaw(String limelightName) {
//         this.limelightName = limelightName;
//     }

//     /**
//      * Calcula o Omega que será enviado para a Swerve.
//      *
//      * @return omega normalizado entre -maxOmega e +maxOmega
//      */
//     public double getOmega() {

//         // Não existe alvo
//         if (!LimelightHelpers.getTV(limelightName)) {
//             return 0.0;
//         }

//         // TX da Limelight
//         double tx = LimelightHelpers.getTX(limelightName);

//         // Já está alinhado
//         if (Math.abs(tx) <= tolerance) {
//             return 0.0;
//         }

//         /*
//          * Controle proporcional:
//          *
//          * erro grande  -> gira rápido
//          * erro pequeno -> gira devagar
//          */
//         double omega = -tx * kP;

//         if (inverted) {
//             omega = -omega;
//         }

//         // Limita a velocidade
//         omega = MathUtil.clamp(
//                 omega,
//                 -maxOmega,
//                 maxOmega
//         );

//         return omega;
//     }

//     /**
//      * Retorna o TX atual.
//      */
//     public double getTX() {

//         if (!LimelightHelpers.getTV(limelightName)) {
//             return 0.0;
//         }

//         return LimelightHelpers.getTX(limelightName);
//     }

//     /**
//      * Retorna true se a câmera estiver vendo uma tag.
//      */
//     public boolean hasTarget() {
//         return LimelightHelpers.getTV(limelightName);
//     }

//     /**
//      * Retorna true quando o robô está alinhado.
//      */
//     public boolean isAligned() {

//         if (!hasTarget()) {
//             return false;
//         }

//         return Math.abs(getTX()) <= tolerance;
//     }

//     public void setKP(double kP) {
//         this.kP = kP;
//     }

//     public void setMaxOmega(double maxOmega) {
//         this.maxOmega = maxOmega;
//     }

//     public void setTolerance(double tolerance) {
//         this.tolerance = tolerance;
//     }

//     public void setInverted(boolean inverted) {
//         this.inverted = inverted;
//     }
// }
