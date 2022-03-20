package frc.robot.subsystems.conveyor.commands.bits;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

public class CheckBeams extends CommandBase {
    private final Conveyor conveyor;
    private boolean lastPreBeam;
    private boolean lastPostBeam;
    private boolean resultPre = false;
    private boolean resultPost = false;

    public CheckBeams(Conveyor conveyor) {
        this.conveyor = conveyor;
    }

    @Override
    public void initialize() {
        lastPreBeam = conveyor.isPreFlapBeamConnected();
        lastPostBeam = conveyor.isPostFlapBeamConnected();
    }

    @Override
    public void execute() {
        if (!resultPre) {
            System.out.println("Still running pre flap beam.");
        }
        if (!resultPost) {
            System.out.println("Still running post flap beam.");
        }

        resultPre |= (lastPreBeam != conveyor.isPreFlapBeamConnected());
        resultPost |= (lastPostBeam != conveyor.isPostFlapBeamConnected());

        lastPreBeam = conveyor.isPreFlapBeamConnected();
        lastPostBeam = conveyor.isPostFlapBeamConnected();
    }

    @Override
    public boolean isFinished() {
        return resultPost && resultPre;
    }
}
