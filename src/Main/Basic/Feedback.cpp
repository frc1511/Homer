#include <Basic/Feedback.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <string>
#include <stdarg.h>

void Feedback::sendString(std::string subsystem, const char* name, std::string_view str) {
    sendString(subsystem, name, str.data());
}

void Feedback::sendString(std::string subsystem, const char* name, const char* format, ...) {
    std::string keyName = subsystem;
    keyName += "_";
    keyName += name;

    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, 256, format, args);
    va_end(args);
    
    frc::SmartDashboard::PutString(keyName.c_str(), buffer);
}

void Feedback::sendDouble(std::string subsystem, const char* name, double value) {
    std::string keyName = subsystem;
    keyName += "_";
    keyName += name;

    frc::SmartDashboard::PutNumber(keyName, value);
}

double Feedback::getDouble(std::string subsystem, const char* name, double fallback) {
    std::string keyName = subsystem;
    keyName += "_";
    keyName += name;
    
    return frc::SmartDashboard::GetNumber(keyName, fallback);
}

void Feedback::sendBoolean(std::string subsystem, const char* name, bool value) {
    sendString(subsystem, name, value ? "true" : "false");
}

void Feedback::sendEditableDouble(std::string subsystem, const char* name, double value) {
    std::string keyName = subsystem;
    keyName += "_";
    keyName += name;
    keyName += "_e";
    frc::SmartDashboard::PutNumber(keyName.c_str(), value);
}

double Feedback::getEditableDouble(std::string subsystem, const char* name, double fallback) {
    std::string keyName = subsystem;
    keyName += "_";
    keyName += name;
    keyName += "_e";

    return frc::SmartDashboard::GetNumber(keyName.c_str(), fallback);
}
