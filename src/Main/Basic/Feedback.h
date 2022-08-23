#pragma once

#include <string>
#include <string_view>

class Feedback {
public:
    /**
     * Sends a string value to the dashboard.
     */
    static void sendString(std::string subsystem, const char* name, std::string_view str);

    /**
     * Sends a string value to the dashboard.
     */
    static void sendString(std::string subsystem, const char* name, const char* fmt, ...);

    /**
     * Sends a double value to the dashboard.
     */
    static void sendDouble(std::string subsystem, const char* name, double value);

    /**
     * Returns a double value from the dashboard.
     */
    static double getDouble(std::string subsystem, const char* name, double fallback);

    /**
     * Sends a boolean value to the dashboard.
     */
    static void sendBoolean(std::string subsystem, const char* name, bool yesno);

    /**
     * Sends an editable double value to the dashboard.
     */
    static void sendEditableDouble(std::string subsystem, const char* name, double value);

    /**
     * Gets the value of an editable double from the dashboard.
     */
    static double getEditableDouble(std::string subsystem, const char* name, double fallback);
};