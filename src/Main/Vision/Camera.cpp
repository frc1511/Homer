#include <Vision/Camera.h>

#define CAMERA_WIDTH 320
#define CAMERA_HEIGHT 240

#define CAMERA_FPS 30
#define CAMERA_EXPOSURE 40

Camera::Camera()
: intakeCamera(frc::CameraServer::StartAutomaticCapture(USB_INTAKE_CAMERA)) {
    
    intakeCamera.SetResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
    intakeCamera.SetFPS(CAMERA_FPS);
    intakeCamera.SetExposureManual(CAMERA_EXPOSURE);

    // Send the video stream to the dashboard.
    frc::CameraServer::PutVideo("intake_camera", CAMERA_WIDTH, CAMERA_HEIGHT);
}

Camera::~Camera() = default;

void Camera::process() { }

void Camera::sendFeedback() { }

void Camera::doPersistentConfiguration() { }

void Camera::resetToMode(MatchMode mode) { }

