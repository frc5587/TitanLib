package org.frc5587.lib;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * LimitedHashMap
 */
public class LimitedHashMap<K, V> extends LinkedHashMap<K, V> {
    private static final long serialVersionUID = -1092490764360856840L;
    private final int maxSize;

    /**
     * Constructs a new {@code LimitedHashMap} with the specified maximum number of
     * entries
     * 
     * @param maxSize the maximum length that the map can reach before values are
     *                discarded
     */
    public LimitedHashMap(int maxSize) {
        this.maxSize = maxSize;
    }

    @Override
    protected boolean removeEldestEntry(Map.Entry<K, V> eldest) {
        return size() > maxSize;
    }
}