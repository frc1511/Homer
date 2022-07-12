#pragma once

#include <map>
#include <optional>

/**
 * Represents a map that can be used for linear interpolation between defined
 * points.
 * 
 * Requires a key type: K, and a value type V.
 */
template<typename K, typename V>
class Interpolation {
public:
    Interpolation()
    : map({}) { }
    
    Interpolation(std::map<K, V> _map)
    : map(_map) { }
    
    /**
     * Inserts a key-value pair into the interpolating tree map.
     */
    void insert(K key, V val) {
        map.insert(std::pair<K, V>(key, val));
    }

    /**
     * Returns an linearly interpolated value in relation to the closest defined
     * points.
     */
    std::optional<V> getInterpolated(K key) const {
        try {
            // Check if the key matches a defined point.
            return map.at(key);
        }
        catch (std::out_of_range&) {
            using iterator = typename std::map<K, V>::const_iterator;
            
            // Iterator to the point directly above.
            const iterator upperBound = map.upper_bound(key);
            // Iterator to the point directly below.
            const iterator lowerBound = --map.lower_bound(key);
            
            // Whether there is no defined point above.
            bool noUpper = (upperBound == map.end());
            // Whether there is no defined point below.
            bool noLower = (lowerBound == map.end());
            
            // Check if there are no defined points.
            if (noUpper && noLower) {
                return {};
            }
            // If there is no defined point above, return the one below because
            // that is the highest one that is defined.
            else if (noUpper) {
                return lowerBound->second;
            }
            // If there is no defined point below, return the one above because
            // that is the lowest one that is defined.
            else if (noLower) {
                return upperBound->second;
            }

            // The upper and lower x values.
            K upperKey = upperBound->first;
            K lowerKey = lowerBound->first;
            // The upper and lower y values.
            V upperVal = upperBound->second;
            V lowerVal = lowerBound->second;

            // Linear interpolation.
            return (((upperVal - lowerVal) / (upperKey - lowerKey)) * (key - lowerKey)) + lowerVal;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; // hi ishan D:D
        }
    }

    /**
     * Returns an linearly interpolated value in relation to the closest defined
     * points.
     */
    std::optional<V> operator[](K key) const {
        return getInterpolated(key);
    }
    
private:
    std::map<K, V> map;
};