//
//  DragFunction.swift
//  swift-ballistics
//
//  Created by Raymond Dowe on 26/11/2024.
//

import Foundation

/// Represents the aerodynamic drag model used to simulate projectile deceleration.
public enum DragFunction: String, Sendable, CaseIterable, Codable {
    /// Standard G1 model (Ingalls), suitable for flat-based, short-ogive sporting projectiles.
    case g1 = "G1"

    /// Standard G7 model, optimized for modern low-drag, boat-tail long-range rifle bullets (VLD).
    case g7 = "G7"
}
