package org.frc5587.lib.advanced;

import java.util.TreeMap;

import edu.wpi.first.wpilibj.geometry.Pose2d;

/**
 * A modified {@link TreeMap} for storing a limited number of timestamps and th
 * 
 * pairs.
 * 
 * <p>
 * LimitedPoseMap clears out the oldest time in the structure when the number of
 * added entries becomes larger than the limit. It also provides a method
 * {@link #getClosest(Double)} for finding the position closest to a moment in
 * time even when there is no exact match, although there is of course no
 * guarantee of position accuracy for these intermediate positions.
 */
public class LimitedPoseMap extends TreeMap<Double, Pose2d> {
    private static final long serialVersionUID = 5949115267041500477L;
    private final int limit;

    /**
     * Makes a new LimitedPoseMap that limits the number of entries to the given
     * limit.
     * 
     * @param limit the number of entries to limit the collection to
     */
    public LimitedPoseMap(int limit) {
        this.limit = limit;
    }

    /**
     * {@inheritDoc}
     * <p>
     * If the addition of the given pair would lead to a number of entries larger
     * than the LimitedPoseMap's {@code limit}, the oldest time and Pose2d pair will
     * be removed to make space for the new pair.
     */
    @Override
    public Pose2d put(Double time, Pose2d pose) {
        if (size() + 1 > limit) {
            remove(firstKey());
        }

        return super.put(time, pose);
    }

    /**
     * Gets the pose for the time closest to the desired time, or null if there or
     * no time and pose pairs logged yet.
     * 
     * <p>
     * Given the limit to the number of entries, the gap between the desired time
     * provided as a parameter and the closest time could be very large. There is
     * also no approximation of intermediate positions when there is no exact match
     * for the given time, so the returned position can vary to a high degree in
     * accuracy.
     * 
     * @param time the time to find the pose closest to, based on previously logged
     *             values
     * @return the logged pose that is closest to the one corresponding to the
     *         provided time
     */
    public Pose2d getClosest(Double time) {
        var low = floorEntry(time);
        var high = ceilingEntry(time);

        if (low != null && high != null) {
            var lowDifference = Math.abs(time - low.getKey());
            var highDifference = Math.abs(time - high.getKey());
            return lowDifference < highDifference ? low.getValue() : high.getValue();
        } else if (low != null || high != null) {
            return low != null ? low.getValue() : high.getValue();
        } else {
            return null;
        }
    }
}