import Combine
import CoreLocation
import Foundation

enum CompassCalibration: Equatable {
    case unavailable    // no magnetometer on this device
    case uncalibrated   // headingAccuracy < 0 (needs calibration)
    case low            // headingAccuracy >= 20°
    case medium         // headingAccuracy 5–20°
    case high           // headingAccuracy < 5°
}

class CompassManager: NSObject, ObservableObject {
    @Published var magneticHeading: Double? = nil
    @Published var calibration: CompassCalibration = .unavailable

    private let locationManager = CLLocationManager()

    override init() {
        super.init()
        guard CLLocationManager.headingAvailable() else { return }
        locationManager.delegate = self
        locationManager.headingFilter = 1.0   // notify on ≥ 1° change
        locationManager.startUpdatingHeading()
        calibration = .uncalibrated
    }
}

extension CompassManager: CLLocationManagerDelegate {
    func locationManager(_ manager: CLLocationManager, didUpdateHeading newHeading: CLHeading) {
        let accuracy = newHeading.headingAccuracy
        DispatchQueue.main.async {
            if accuracy < 0 {
                self.calibration = .uncalibrated
                self.magneticHeading = nil
            } else {
                self.magneticHeading = newHeading.magneticHeading
                self.calibration = accuracy < 5 ? .high : accuracy < 20 ? .medium : .low
            }
        }
    }

    // Show the system calibration UI whenever we haven't converged yet.
    func locationManagerShouldDisplayHeadingCalibration(_ manager: CLLocationManager) -> Bool {
        return calibration == .uncalibrated
    }
}
