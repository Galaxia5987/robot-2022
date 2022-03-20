package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreaker {
    private final DigitalInput beam;
    private boolean hadObjectLastLoop = false;
    private boolean hasObject;

    public BeamBreaker(int beamBreakerPort) {
        this.beam = new DigitalInput(beamBreakerPort);
    }

    public boolean hasObject() {
        return !get();
    }

    /**
     * Updates value of beam breaker.
     * Call this every loop, unless you're a psycho.
     */
    public void updateBeamBreaker() {
        hadObjectLastLoop = hasObject;
        hasObject = hasObject();
    }

    public boolean get() {
        return beam.get();
    }

    public boolean hasChanged() {
        return hasObject != hadObjectLastLoop;
    }
}
