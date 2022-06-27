package frc.robot.lib.math;

/** A simple class to wrap gear calculations. */
public class Gearbox {
    
    /** The gearbox ratio. */
    private double m_ratio;

    /** Constructs a Gearbox directly from the gearbox ratio. */
    public Gearbox(double ratio) {
        m_ratio = ratio;
    }

    /** Constructs a Gearbox from the sizes of the driven and driving gears. */
    public Gearbox(double driven, double driving) {
        m_ratio = driven / driving;
    }

    public double getRatio() {
        return m_ratio;
    }

    public double drivenRotationToDriving(double rot) {
        return rot / m_ratio;
    }

    public double drivingRotationToDriven(double rot) {
        return rot * m_ratio;
    }

}
