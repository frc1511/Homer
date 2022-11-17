#pragma once

#include <map>
#include <optional>
#include <stdexcept>

/**
 * Represents a map that can be used for linear interpolation between defined
 * points.
 * 
 * Key type K and value type V.
 */
template<typename K, typename V>
class Interpolation {
public:
    Interpolation()
    : map() { }
    
    Interpolation(std::map<K, V> _map)
    : map(_map) { }
    
    /**
     * Inserts a key-value pair into the interpolating tree map.
     */
    void insert(K key, V val) {
        map.emplace(key, val);
    }

    /**
     * Returns an linearly interpolated value between the closest defined points.
     */
    std::optional<V> getInterpolated(K key) const {
        try {
            // Check if the key matches a defined point.
            return map.at(key);
        }
        catch (std::out_of_range&) {
            // Iterator to the point directly above and below the key.
            typename decltype(map)::const_iterator upperBound = map.upper_bound(key),
                                                   lowerBound = --map.lower_bound(key);
            
            // Whether there is no defined point above or below the key.
            bool noUpper = (upperBound == map.cend()),
                 noLower = (lowerBound == map.cend());
            
            // No defined points D:
            if (noUpper && noLower) {
                return {};
            }
            // Return the highest defined point if there is no upper bound.
            else if (noUpper) {
                return lowerBound->second;
            }
            // Return the lowest defined point if there is no lower bound.
            else if (noLower) {
                return upperBound->second;
            }

            // The upper x and y values.
            auto [upperKey, upperVal] = *upperBound;
            // The lower x and y values.
            auto [lowerKey, lowerVal] = *lowerBound;

            // Linear interpolation.
            return (((upperVal - lowerVal) / (upperKey - lowerKey)) * (key - lowerKey)) + lowerVal;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; // hi ishan D:D
        }
    }

    /**
     * Returns an linearly interpolated value between the closest defined points.
     */
    std::optional<V> operator[](K key) const {
        return getInterpolated(key);
    }
    
private:
    std::map<K, V> map;
};