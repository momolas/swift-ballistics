//
//  PBRTests.swift
//  swift-ballistics
//
//  Created by Jules on 26/02/2025.
//

import Testing
@testable import Ballistics

@Test func calculatePBR() async throws {
    let result = PBR.solve(
        dragCoefficient: 0.414,
        initialVelocity: 3000,
        sightHeight: 1.5,
        vitalSize: 8 // 8 inch target
    )

    switch result {
    case .success(let pbr):
        #expect(pbr.nearZeroYards > 0)
        #expect(pbr.farZeroYards > pbr.nearZeroYards)
        #expect(pbr.maxPBRYards > pbr.farZeroYards)
        #expect(pbr.minPBRYards >= 0)

        // Sanity checks for a 3000 fps projectile
        // Usually near zero is around 30-50 yards
        // Far zero around 200-250 yards
        // Max PBR around 280-300 yards
        #expect(pbr.nearZeroYards < 100)
        #expect(pbr.farZeroYards > 100)
        #expect(pbr.maxPBRYards > 200)

    case .failure(let error):
        Issue.record("PBR calculation failed with error: \(error)")
    }
}
