package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetPoseElevCommand extends Command{

    private ElevatorSubsystem m_Elevator = RobotContainer.elevatorSubsystem;
    private double UpDown;
    private boolean conditionEnd;
    private boolean teste = false;
    private double setPose = 0;

    public SetPoseElevCommand(double UpDown) {
        this.UpDown = UpDown;

        addRequirements(m_Elevator);
    }

    @Override
    public void initialize() {
        conditionEnd = false;
        setPose = m_Elevator.getPose_Elev();
    }

    @Override
    public void execute() {
        teste = !teste;

        setPose += (UpDown/8);
        setPose = MathUtil.clamp(setPose, -174, 0);

        SmartDashboard.putNumber("setPose", setPose);
        
        if(m_Elevator.CanMove() && ArmSubsystem.L1) {
            m_Elevator.setPose_Elev(setPose);
        } else {
            m_Elevator.freeze();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_Elevator.freeze();
    }
}
