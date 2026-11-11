package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class XboxControls {
    private final static XboxController m_controller = new XboxController(0);
    
        public XboxControls() {
            
        }
        public final double Get_RightX() {
            return m_controller.getRightX();
        }
        public final double Get_LeftX() {
            return m_controller.getLeftX();
        }
        public final double Get_LeftY() {
            return m_controller.getLeftY();
        }
    
        public final boolean Get_Start() {
            return m_controller.getStartButtonPressed();
        }
    
        public final boolean Get_Abutton() {
        
            return m_controller.getAButtonPressed();
            
        }
    
        public final static double Get_RightTriggerAxis() {
            return m_controller.getRightTriggerAxis();
    }

    public final boolean Get_RightBumper() {
        return m_controller.getRightBumper();
    }
    
}
