//
//  BPR.swift
//  swift-ballistics
//
//  Created by Raymond Dowe on 26/11/2024.
//

import Foundation

struct PBR {
    
    var nearZeroYards: Int
    var farZeroYards: Int
    var minPBRYards: Int
    var maxPBRYards: Int
    var sightInAt100Yards: Int
    
    func PBRGetNearZeroYards(pbr: PBR) -> Int {
        return pbr.nearZeroYards
    }
    
    func PBRGetFarZeroYards(pbr: PBR) -> Int {
        return pbr.farZeroYards
    }
    
    func PBRGetMinPBRYards(pbr: PBR) -> Int {
        return pbr.minPBRYards
    }
    
    func PBRGetMaxPBRYards(pbr: PBR) -> Int {
        return pbr.maxPBRYards
    }
    
    func PBRGetSightInAt100Yards(pbr: PBR) -> Int {
        return pbr.sightInAt100Yards
    }
    
    func solvePBR(
        dragCoefficient: Double,
        initialVelocity: Double,
        sightHeight: Double,
        vitalSize: Double
    ) -> Result<PBR, Error> {

        var zero: Double = -1
        var farZero: Double = 0
        var vertexKeep = false
        var yVertex: Double = 0
        var minPBRRange: Double = 0
        var minPBRKeep = false
        var maxPBRRange: Double = 0
        var maxPBRKeep = false
        var tin100: Int = 0
        
        let shootingAngle: Double = 0
        var zAngle: Double = 0
        var step: Double = 10
        var quit = false
        
        while !quit {
            let Gy = Constants.gravity * cos(Math.degToRad(shootingAngle + zAngle))
            let Gx = Constants.gravity * sin(Math.degToRad(shootingAngle + zAngle))
            let vxInit = initialVelocity * cos(Math.degToRad(zAngle))
            let vyInit = initialVelocity * sin(Math.degToRad(zAngle))
            
            var x: Double = 0
            var y: Double = -sightHeight / 12
            var vx = vxInit
            var vy = vyInit
            var keep = false
            var keep2 = false
            var tinkeep = false
            minPBRKeep = false
            maxPBRKeep = false
            vertexKeep = false
            
            var status: Int = 0
            let dt: Double = 0.5 / initialVelocity
            
            while true {
                let v = sqrt(vx * vx + vy * vy)
                let dv = Drag.retard(dragCoefficient: dragCoefficient, projectileVelocity: v)
                let dvx = -(vx / v) * dv
                let dvy = -(vy / v) * dv
                
                let vxNext = vx + dt * dvx + dt * Gx
                let vyNext = vy + dt * dvy + dt * Gy
                
                x += dt * (vx + vxNext) / 2
                y += dt * (vy + vyNext) / 2
                
                vx = vxNext
                vy = vyNext
                
                if y > 0 && !keep && vy >= 0 {
                    zero = x
                    keep = true
                }
                
                if y < 0 && !keep2 && vy <= 0 {
                    farZero = x
                    keep2 = true
                }
                
                if (12 * y) > -(vitalSize / 2) && !minPBRKeep {
                    minPBRRange = x
                    minPBRKeep = true
                }
                
                if (12 * y) < -(vitalSize / 2) && minPBRKeep && !maxPBRKeep {
                    maxPBRRange = x
                    maxPBRKeep = true
                }
                
                if x >= 300 && !tinkeep {
                    tin100 = Int(100 * y * 12)
                    tinkeep = true
                }
                
                if abs(vy) > abs(3 * vx) {
                    status = 1 // Error: Too fast vertical velocity
                    break
                }

                if vy < 0 && !vertexKeep {
                    yVertex = y
                    vertexKeep = true
                }
                
                if keep && keep2 && minPBRKeep && maxPBRKeep && vertexKeep && tinkeep {
                    break
                }
            }
            
            if status != 0 {
                return .failure(NSError(domain: "PBRComputation", code: status, userInfo: nil))
            }
            
            if (yVertex * 12) > (vitalSize / 2) {
                step = -abs(step) / 2 // Vertex too high
            } else if (yVertex * 12) <= (vitalSize / 2) {
                step = abs(step) / 2 // Vertex too low
            }
            
            zAngle += step
            
            if abs(step) < (0.01 / 60) {
                quit = true
            }
        }
        
        let result = PBR(
            nearZeroYards: Int(zero / 3),
            farZeroYards: Int(farZero / 3),
            minPBRYards: Int(minPBRRange / 3),
            maxPBRYards: Int(maxPBRRange / 3),
            sightInAt100Yards: tin100
        )
        
        return .success(result)
    }
}
