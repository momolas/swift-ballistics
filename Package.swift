// swift-tools-version: 6.0
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
    name: "swift-ballistics",
    platforms: [.iOS(.v13), .macOS(.v10_15), .watchOS(.v6)],
    products: [
        .library(
            name: "Ballistics",
            targets: ["Ballistics"]
        )
    ],
    dependencies: [
        .package(url: "https://github.com/apple/swift-numerics.git", .upToNextMajor(from: "1.0.0"))
    ],
    targets: [
        .target(
            name: "Ballistics",
            dependencies: []
        ),
        .testTarget(
            name: "BallisticsTests",
            dependencies: [
                "Ballistics",
                .product(name: "Numerics", package: "swift-numerics")
            ]
        ),
    ]
)
