#pragma once

#include <string_view>

class Feedback {
public:
    /**
     * Sends a string value to the dashboard.
     */
    static void sendString(const char* subsystem, std::string_view name, std::string_view str);

    /**
     * Sends a string value to the dashboard.
     */
    static void sendString(const char* subsystem, std::string_view name, const char* fmt, ...);

    /**
     * Sends a double value to the dashboard.
     */
    static void sendDouble(const char* subsystem, std::string_view name, double value);

    /**
     * Returns a double value from the dashboard.
     */
    static double getDouble(const char* subsystem, std::string_view name, double fallback);

    /**
     * Sends a boolean value to the dashboard.
     */
    static void sendBoolean(const char* subsystem, std::string_view name, bool yesno);

    /**
     * Sends an editable double value to the dashboard.
     */
    static void sendEditableDouble(const char* subsystem, std::string_view name, double value);

    /**
     * Gets the value of an editable double from the dashboard.
     */
    static double getEditableDouble(const char* subsystem, std::string_view name, double fallback);
};