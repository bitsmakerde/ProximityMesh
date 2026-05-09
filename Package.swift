// swift-tools-version: 6.0
import PackageDescription

let package = Package(
    name: "ProximityMesh",
    platforms: [.visionOS(.v2)],
    products: [
        .library(name: "ProximityMesh", targets: ["ProximityMesh"]),
    ],
    targets: [
        .target(name: "ProximityMesh"),
    ]
)
