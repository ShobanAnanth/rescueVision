import SwiftUI
import ARKit

struct ContentView: View {
    @EnvironmentObject var ble: BLEManager
    @EnvironmentObject var ni: NIManager
    @EnvironmentObject var ar: ARManager
    @EnvironmentObject var estimator: AnchorEstimator

    var body: some View {
        ZStack {
            ARViewContainer(arManager: ar, estimator: estimator)
                .ignoresSafeArea()

            if let angle = estimator.offScreenAngle {
                Image(systemName: "location.north.fill")
                    .resizable()
                    .aspectRatio(contentMode: .fit)
                    .frame(width: 40, height: 40)
                    .foregroundColor(.red)
                    .shadow(radius: 5)
                    // The image points "Up".
                    // SwiftUI rotates clockwise. Angle is mathematically CCW.
                    // If angle = 0 (right), we need it to point Right, so rotate 90 deg clockwise.
                    // Actually, if image points UP, we rotate by PI/2 - angle.
                    .rotationEffect(.radians(.pi / 2 - angle))
                    // Offset moves it towards the edge
                    .offset(x: cos(angle) * 140, y: -sin(angle) * 140)
                    .animation(.interactiveSpring(), value: angle)
            }

            VStack {
                HUDView(ble: ble, ni: ni, estimator: estimator)
                    .padding()
                Spacer()
                Button("Reset Estimate") {
                    estimator.reset()
                }
                .buttonStyle(.borderedProminent)
                .tint(.red.opacity(0.8))
                .padding(.bottom, 40)
            }
        }
    }
}

// MARK: - HUD overlay

private struct HUDView: View {
    @ObservedObject var ble: BLEManager
    @ObservedObject var ni: NIManager
    @ObservedObject var estimator: AnchorEstimator

    var body: some View {
        VStack(alignment: .leading, spacing: 6) {
            HUDRow(label: "BLE", value: bleStateText)
            HUDRow(label: "NI", value: niStateText)
            HUDRow(label: "Range", value: rangeText)
            HUDRow(label: "Measurements", value: "\(estimator.measurementCount)")
            HUDRow(label: "Residual", value: String(format: "%.3f m", estimator.residualError))
        }
        .padding(12)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
        .frame(maxWidth: .infinity, alignment: .leading)
    }

    private var bleStateText: String {
        switch ble.connectionState {
        case .disconnected: return "Disconnected"
        case .scanning:     return "Scanning…"
        case .connecting:   return "Connecting…"
        case .connected:    return "Connected"
        }
    }

    private var niStateText: String {
        switch ni.sessionState {
        case .idle:                 return "Idle"
        case .waitingForAccessory:  return "Waiting for board…"
        case .configuring:          return "Configuring…"
        case .ranging:              return "Ranging"
        case .error(let msg):       return "Error: \(msg)"
        }
    }

    private var rangeText: String {
        if let r = ni.lastRange {
            return String(format: "%.2f m", r)
        }
        return "—"
    }
}

private struct HUDRow: View {
    let label: String
    let value: String

    var body: some View {
        HStack {
            Text(label)
                .font(.caption.bold())
                .foregroundStyle(.secondary)
                .frame(width: 100, alignment: .leading)
            Text(value)
                .font(.caption)
                .foregroundStyle(.primary)
        }
    }
}

#Preview {
    ContentView()
        .environmentObject(BLEManager())
        .environmentObject(NIManager())
        .environmentObject(ARManager())
        .environmentObject(AnchorEstimator())
}
