package frc.robot.utilities;

import java.util.Collection;
import java.util.Comparator;
import java.util.Map;
import java.util.NavigableMap;
import java.util.NavigableSet;
import java.util.Set;
import java.util.SortedMap;

public class InterpolatedLookupTable<X, Y> implements NavigableMap<X, Y> {

    @FunctionalInterface
    public abstract interface Interpolator<X, Y> {
        public abstract Y interpolate(X t, X x1, Y y1, X x2, Y y2);

        public default Y interpolate(X t, Entry<X, Y> p1, Entry<X, Y> p2) {
            return interpolate(t, p1.getKey(), p1.getValue(), p2.getKey(), p2.getValue());
        }
    }

    private NavigableMap<X, Y> samples;
    private Interpolator<X, Y> interpolator;

    public InterpolatedLookupTable(NavigableMap<X, Y> samples, Interpolator<X, Y> interpolator) {
        this.samples = samples;
        this.interpolator = interpolator;
    }

    @Override
    @SuppressWarnings("unchecked")
    public Y get(Object key) {
        if (samples.containsKey(key)) {
            return samples.get(key);
        } else {
            Entry<X, Y> lowerEntry = samples.lowerEntry((X) key);
            Entry<X, Y> higherEntry = samples.higherEntry((X) key);
            System.out.println(key);
            System.out.println(lowerEntry);
            System.out.println(higherEntry);

            if (lowerEntry == null || higherEntry == null) {
                return null;
            } else {
                return interpolator.interpolate((X) key, lowerEntry, higherEntry);
            }
        }
    }

    @Override
    @SuppressWarnings("unchecked")
    public boolean containsKey(Object key) {
        return comparator().compare(firstKey(), (X) key) > 0 && comparator().compare((X) key, lastKey()) < 0;
    }

    @Override
    public boolean containsValue(Object value) {
        throw new UnsupportedOperationException("Cannot call containsValue with an unspecified range.");
    }

    @Override
    public void clear() {
        samples.clear();
    }

    @Override
    public Set<Entry<X, Y>> entrySet() {
        return samples.entrySet();
    }

    @Override
    public boolean isEmpty() {
        return samples.isEmpty();
    }

    @Override
    public Set<X> keySet() {
        return samples.keySet();
    }

    @Override
    public Y put(X x, Y y) {
        return samples.put(x, y);
    }

    @Override
    public void putAll(Map<? extends X, ? extends Y> map) {
        samples.putAll(map);        
    }

    @Override
    public Y remove(Object key) {
        return samples.remove(key);
    }

    @Override
    public int size() {
        return samples.size();
    }

    @Override
    public Collection<Y> values() {
        return samples.values();
    }

    @Override
    public Comparator<? super X> comparator() {
        return samples.comparator();
    }

    @Override
    public X firstKey() {
        return samples.firstKey();
    }

    @Override
    public X lastKey() {
        return samples.lastKey();
    }

    @Override
    public Entry<X, Y> ceilingEntry(X key) {
        return samples.ceilingEntry(key);
    }

    @Override
    public X ceilingKey(X key) {
        return samples.ceilingKey(key);
    }

    @Override
    public NavigableSet<X> descendingKeySet() {
        return samples.descendingKeySet();
    }

    @Override
    public NavigableMap<X, Y> descendingMap() {
        return samples.descendingMap();
    }

    @Override
    public Entry<X, Y> firstEntry() {
        return samples.firstEntry();
    }

    @Override
    public Entry<X, Y> floorEntry(X key) {
        return samples.floorEntry(key);
    }

    @Override
    public X floorKey(X key) {
        return samples.floorKey(key);
    }

    @Override
    public SortedMap<X, Y> headMap(X toKey) {
        return samples.headMap(toKey);
    }

    @Override
    public NavigableMap<X, Y> headMap(X arg0, boolean arg1) {
        return samples.headMap(arg0, arg1);
    }

    @Override
    public Entry<X, Y> higherEntry(X key) {
        return samples.higherEntry(key);
    }

    @Override
    public X higherKey(X key) {
        return samples.higherKey(key);
    }

    @Override
    public Entry<X, Y> lastEntry() {
        return samples.lastEntry();
    }

    @Override
    public Entry<X, Y> lowerEntry(X key) {
        return samples.lowerEntry(key);
    }

    @Override
    public X lowerKey(X key) {
        return samples.lowerKey(key);
    }

    @Override
    public NavigableSet<X> navigableKeySet() {
        return samples.navigableKeySet();
    }

    @Override
    public Entry<X, Y> pollFirstEntry() {
        return samples.pollFirstEntry();
    }

    @Override
    public Entry<X, Y> pollLastEntry() {
        return samples.pollLastEntry();
    }

    @Override
    public SortedMap<X, Y> subMap(X arg0, X arg1) {
        return samples.subMap(arg0, arg1);
    }

    @Override
    public NavigableMap<X, Y> subMap(X arg0, boolean arg1, X arg2, boolean arg3) {
        return samples.subMap(arg0, arg1, arg2, arg3);
    }

    @Override
    public SortedMap<X, Y> tailMap(X fromKey) {
        return samples.tailMap(fromKey);
    }

    @Override
    public NavigableMap<X, Y> tailMap(X arg0, boolean arg1) {
        return samples.tailMap(arg0, arg1);
    }
    
}
