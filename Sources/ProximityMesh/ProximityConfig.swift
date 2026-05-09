import Foundation
import simd

/// Configuration for the proximity mesh warning system.
public struct ProximityConfig: Sendable {
    /// Distance in meters at which objects start being visualized (default: 0.5m).
    public var warningDistance: Float

    /// Size of the visible area around the user in meters (default: 1.0m).
    public var visualizationExtent: Float

    /// Radius of each point sphere in meters (default: 0.0035m / 3.5mm).
    public var pointRadius: Float

    /// Distance between generated surface points in meters (default: 0.04m / 4cm).
    public var surfacePointSpacing: Float

    /// Maximum number of subdivisions used when resampling a single mesh face.
    public var maxFaceSampleDivisions: Int

    /// RGB color of the warning points (default: red).
    public var pointColor: SIMD3<Float>

    /// Opacity of the warning points (default: 0.8).
    public var pointOpacity: Float

    /// Maximum number of points to display simultaneously (default: 8000).
    public var maxPointCount: Int

    /// Interval between visualization updates in seconds (default: 0.1 / 10Hz).
    public var updateInterval: TimeInterval

    /// Whether to include floor surfaces in the visualization (default: false).
    public var showFloor: Bool

    /// Whether to include ceiling surfaces in the visualization (default: false).
    public var showCeiling: Bool

    /// Optional maximum world-space height for visible samples.
    public var maximumSurfaceHeight: Float?

    /// Whether to show only real-world samples visually overlapped by registered CAD entities.
    public var requireCADOverlap: Bool

    /// Whether to keep only samples in the current view direction.
    public var filterToViewDirection: Bool

    /// Cone angle in degrees used when filtering samples to the current view direction.
    public var viewConeDegrees: Float

    /// Expansion added to occluding entity bounds in meters (default: 0.03m / 3cm).
    public var occlusionPadding: Float

    @available(*, deprecated, renamed: "requireCADOverlap")
    public var onlyShowOccludedPoints: Bool {
        get { requireCADOverlap }
        set { requireCADOverlap = newValue }
    }

    public init(
        warningDistance: Float = 0.5,
        visualizationExtent: Float = 1.0,
        pointRadius: Float = 0.0035,
        surfacePointSpacing: Float = 0.04,
        maxFaceSampleDivisions: Int = 16,
        pointColor: SIMD3<Float> = SIMD3<Float>(1.0, 0.3, 0.3),
        pointOpacity: Float = 0.8,
        maxPointCount: Int = 8000,
        updateInterval: TimeInterval = 0.1,
        showFloor: Bool = false,
        showCeiling: Bool = false,
        maximumSurfaceHeight: Float? = nil,
        requireCADOverlap: Bool = false,
        filterToViewDirection: Bool = false,
        viewConeDegrees: Float = 95,
        onlyShowOccludedPoints: Bool? = nil,
        occlusionPadding: Float = 0.03
    ) {
        self.warningDistance = warningDistance
        self.visualizationExtent = visualizationExtent
        self.pointRadius = pointRadius
        self.surfacePointSpacing = surfacePointSpacing
        self.maxFaceSampleDivisions = maxFaceSampleDivisions
        self.pointColor = pointColor
        self.pointOpacity = pointOpacity
        self.maxPointCount = maxPointCount
        self.updateInterval = updateInterval
        self.showFloor = showFloor
        self.showCeiling = showCeiling
        self.maximumSurfaceHeight = maximumSurfaceHeight
        self.requireCADOverlap = onlyShowOccludedPoints ?? requireCADOverlap
        self.filterToViewDirection = filterToViewDirection
        self.viewConeDegrees = viewConeDegrees
        self.occlusionPadding = occlusionPadding
    }
}
