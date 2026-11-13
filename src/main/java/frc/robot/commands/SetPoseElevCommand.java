package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetPoseElevCommand extends Command{

    private ElevatorSubsystem m_Elevator = RobotContainer.elevatorSubsystem;
    private double UpDown;
    private boolean conditionEnd;

    public SetPoseElevCommand(double UpDown) {
        this.UpDown = UpDown;

        addRequirements(m_Elevator);
    }

    @Override
    public void initialize() {
        conditionEnd = false;
    }

    @Override
    public void execute() {
        if(m_Elevator.CanMove()) {
            m_Elevator.setVel_Elev(UpDown);
        } else {
            conditionEnd = true;
        }
    }

    @Override
    public boolean isFinished() {
        return conditionEnd;
    }

    @Override
    public void end(boolean interrupted) {
        m_Elevator.freeze();
    }
}
