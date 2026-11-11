
// package frc.robot.SwerveDrive;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.LimelightHelpers;

// public class AutonomoTag {

//     // ============================================================
//     // APRILTAG YAW
//     // ============================================================

//     // Classe responsável pelo cálculo do alinhamento através do TX
//     private final AprilTagYaw aprilTagYaw =
//             new AprilTagYaw("limelight");

//     // Velocidade de rotação utilizada para procurar a AprilTag
//     private static final double PROCURA_OMEGA = 0.40;


//     // ============================================================
//     // ALTURAS
//     // ============================================================

//     // Altura da AprilTag em metros
    
//     private double altTag = 0.385;

//     // Altura da câmera em metros
//     private double altCam = 0.520;

//     // Diferença entre as alturas
//     private double deltaAlt = altTag - altCam;


//     // ============================================================
//     // DISTÂNCIA
//     // ============================================================

//     // Distância calculada entre a câmera e a AprilTag
//     private double distance = 0;

//     // Distância desejada até a AprilTag
//     private double distanceDesejada = 0;


//     // ============================================================
//     // CONSTRUTOR
//     // ============================================================

//     public AutonomoTag() {

//     }


//     // ============================================================
//     // CONTROLE DE ALINHAMENTO
//     // ============================================================

//     public double getOmega() {

//         // Verifica se a Limelight está detectando uma AprilTag
//         if (aprilTagYaw.hasTarget()) {

//             // Retorna a velocidade angular calculada pelo controle
//             return -aprilTagYaw.getOmega();

//         } else {

//             // Procura a AprilTag quando ela não está visível
//             return PROCURA_OMEGA;
//         }
//     }


//     // ============================================================
//     // DISTÂNCIA DA APRILTAG
//     // ============================================================

//     public double Distance() {

//         // Ângulo vertical entre a câmera e a AprilTag
//         double TyCam = LimelightHelpers.getTY("limelight");


//         // Se não existe uma AprilTag detectada, retorna zero
//         if (!aprilTagYaw.hasTarget()) {

//             distance = 0;

//             return distance;
//         }


//         // Converte o ângulo de graus para radianos
//         double anguloTotal = Math.toRadians(TyCam);


//         // Evita divisão por um valor muito próximo de zero
//         if (Math.abs(Math.tan(anguloTotal)) < 0.001) {

//             distance = 0;

//             return distance;
//         }


//         // Calcula a distância através da diferença de alturas
//         distance = deltaAlt / Math.tan(anguloTotal);


//         // Calibração da distância Apriltag
//         distance = 1.0222 * distance - 0.1497;


//         return distance;
//     }


//     // ============================================================
//     //                      SMARTDASHBOARD
//     // ============================================================

//     public void dashboard() {

//         // Informa se a AprilTag está sendo detectada
//         SmartDashboard.putBoolean(
//                 "Apriltag Detectada",
//                 aprilTagYaw.hasTarget()
//         );


//         // Ângulo horizontal da AprilTag
//         SmartDashboard.putNumber(
//                 "Apriltag TX",
//                 aprilTagYaw.getTX()
//         );


//         // Ângulo vertical da AprilTag
//         SmartDashboard.putNumber(
//                 "AprilTag TY",
//                 LimelightHelpers.getTY("limelight")
//         );


//         // Distância calculada até a AprilTag
//         SmartDashboard.putNumber(
//                 "DISTANCE (m)",
//                 Distance()
//         );
//     }


//     // ============================================================
//     //                  VERIFICAÇÃO DA APRILTAG
//     // ============================================================

//     public boolean hasTarget() {

//         return aprilTagYaw.hasTarget();
//     }


//     // ============================================================
//     //                      LEITURA DO TX
//     // ============================================================

//     public double getTX() {

//         return aprilTagYaw.getTX();
//     }

// }

