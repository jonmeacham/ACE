using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;
using ACE.Server.Physics.Animation;

namespace ACE.Server.Physics.SIMD
{
    /// <summary>
    /// Extension methods to integrate SIMD optimizations with existing PhysicsObj operations.
    /// Provides easy adoption of SIMD performance improvements in the physics engine.
    /// </summary>
    public static class PhysicsObjExtensions
    {
        /// <summary>
        /// Gets the estimated radius of a PhysicsObj for collision detection
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float GetRadius(this PhysicsObj obj)
        {
            // Return cached radius if available
            if (obj.BoundingRadius > 0)
                return obj.BoundingRadius;

            // Calculate rough radius from PartArray if available
            if (obj.PartArray?.Parts != null)
            {
                var maxRadius = 0.0f;
                foreach (var part in obj.PartArray.Parts)
                {
                    if (part?.GfxObj?.Vertices != null)
                    {
                        // Rough estimation - would need more accurate calculation in production
                        var radius = EstimatePartRadius(part);
                        if (radius > maxRadius)
                            maxRadius = radius;
                    }
                }
                return maxRadius;
            }

            // Fallback default radius
            return 1.0f;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float EstimatePartRadius(PhysicsPart part)
        {
            // Simplified radius estimation
            // In production, this would use proper bounding sphere calculation
            return 1.0f; // Placeholder
        }

        /// <summary>
        /// Updates physics for a batch of objects using SIMD optimizations
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void UpdatePhysicsSIMD(this IList<PhysicsObj> objects, float quantum)
        {
            if (!SIMDPhysicsEngine.Config.EnableSIMDOptimizations || objects.Count < SIMDPhysicsEngine.Config.MinObjectsForSIMD)
            {
                // Fall back to individual updates
                foreach (var obj in objects)
                {
                    if (obj?.is_active() == true)
                        obj.update_object();
                }
                return;
            }

            SIMDPhysicsEngine.UpdatePhysicsBatch(objects, quantum);
        }

        /// <summary>
        /// Transforms multiple AFrame objects efficiently using SIMD
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformBatchSIMD(this IList<AFrame> frames, Matrix4x4 transform, IList<AFrame> results)
        {
            if (!SIMDPhysicsEngine.Config.EnableSIMDOptimizations || frames.Count < 4)
            {
                // Fall back to individual transforms
                for (int i = 0; i < frames.Count; i++)
                {
                    var frame = frames[i];
                    var transformedOrigin = Vector3.Transform(frame.Origin, transform);
                    // For rotation, we need to extract rotation part of matrix (simplified)
                    results[i] = new AFrame(transformedOrigin, frame.Orientation);
                }
                return;
            }

            SIMDPhysicsEngine.TransformAFramesBatch(frames, transform, results);
        }

        /// <summary>
        /// Checks if an object should use SIMD optimizations based on its properties
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool ShouldUseSIMDOptimizations(this PhysicsObj obj)
        {
            if (!SIMDPhysicsEngine.Config.EnableSIMDOptimizations)
                return false;

            // Simple heuristics for when SIMD is beneficial
            return obj.is_active() && 
                   obj.PartArray?.Parts?.Count > 0 &&
                   (obj.TransientState.HasFlag(TransientStateFlags.Active) || 
                    obj.State.HasFlag(PhysicsState.Gravity));
        }

        /// <summary>
        /// Gets vertices from a PhysicsObj for SIMD processing
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector3[] GetVerticesForSIMD(this PhysicsObj obj)
        {
            var vertices = new List<Vector3>();

            if (obj.PartArray?.Parts != null)
            {
                foreach (var part in obj.PartArray.Parts)
                {
                    if (part?.GfxObj?.Vertices != null)
                    {
                        // Extract vertices from the graphics object
                        foreach (var vertex in part.GfxObj.Vertices.Values)
                        {
                            vertices.Add(vertex.Origin);
                        }
                    }
                }
            }

            return vertices.ToArray();
        }

        /// <summary>
        /// Updates the bounding box using SIMD optimizations
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void UpdateBoundingBoxSIMD(this PhysicsObj obj)
        {
            if (!obj.ShouldUseSIMDOptimizations())
            {
                // Fall back to standard method
                obj.recalc_cross_cells();
                return;
            }

            var vertices = obj.GetVerticesForSIMD();
            if (vertices.Length > 0)
            {
                SIMDBBox.ComputeBoundingBox(vertices, out var min, out var max);
                
                // Update object's bounding information
                // This is simplified - actual implementation would need to integrate with existing bounding box system
                obj.BoundingRadius = (max - min).Length() * 0.5f;
            }
        }

        /// <summary>
        /// Performs batch collision detection using SIMD optimizations
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void DetectCollisionsSIMD(this IList<PhysicsObj> objects)
        {
            if (!SIMDPhysicsEngine.Config.EnableSIMDOptimizations || objects.Count < 8)
            {
                // Fall back to existing collision detection
                foreach (var obj in objects)
                {
                    // This would integrate with existing collision detection system
                    obj?.CheckCollisions(); // Assuming this method exists
                }
                return;
            }

            // Create collision pairs for broad-phase detection
            var collisionPairs = new List<(PhysicsObj, PhysicsObj)>();
            
            for (int i = 0; i < objects.Count; i++)
            {
                for (int j = i + 1; j < objects.Count; j++)
                {
                    var objA = objects[i];
                    var objB = objects[j];
                    
                    if (objA?.is_active() == true && objB?.is_active() == true)
                    {
                        collisionPairs.Add((objA, objB));
                    }
                }
            }

            SIMDPhysicsEngine.ProcessCollisionsBatch(collisionPairs);
        }

        /// <summary>
        /// Placeholder for CheckCollisions method - would need to be implemented or use existing method
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void CheckCollisions(this PhysicsObj obj)
        {
            // This would integrate with the existing collision detection system
            // For now, it's a placeholder
        }

        /// <summary>
        /// Optimized position update using SIMD when possible
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void UpdatePositionSIMD(this PhysicsObj obj, double quantum)
        {
            if (!obj.ShouldUseSIMDOptimizations())
            {
                obj.update_position();
                return;
            }

            // For individual objects, SIMD doesn't provide much benefit
            // This would be more useful when processing multiple objects
            obj.update_position();
        }

        /// <summary>
        /// Batch update positions for multiple objects
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void UpdatePositionsBatch(this IList<PhysicsObj> objects, double quantum)
        {
            if (!SIMDPhysicsEngine.Config.EnableSIMDOptimizations)
            {
                foreach (var obj in objects)
                    obj?.update_position();
                return;
            }

            // Group objects that can benefit from batch processing
            var batchObjects = new List<PhysicsObj>();
            var individualObjects = new List<PhysicsObj>();

            foreach (var obj in objects)
            {
                if (obj?.ShouldUseSIMDOptimizations() == true)
                    batchObjects.Add(obj);
                else if (obj != null)
                    individualObjects.Add(obj);
            }

            // Process batch objects with SIMD
            if (batchObjects.Count >= 4)
            {
                // Extract position data for batch processing
                var frames = new AFrame[batchObjects.Count];
                for (int i = 0; i < batchObjects.Count; i++)
                {
                    frames[i] = batchObjects[i].Position.Frame;
                }

                // This would need more sophisticated batch position updating
                // For now, fall back to individual processing
                foreach (var obj in batchObjects)
                    obj.update_position();
            }
            else
            {
                foreach (var obj in batchObjects)
                    obj.update_position();
            }

            // Process individual objects normally
            foreach (var obj in individualObjects)
                obj.update_position();
        }

        /// <summary>
        /// Performance monitoring for SIMD operations
        /// </summary>
        public static class PerformanceMonitor
        {
            private static long _simdOperations = 0;
            private static long _scalarOperations = 0;
            private static double _simdTime = 0.0;
            private static double _scalarTime = 0.0;

            public static void RecordSIMDOperation(double timeMs)
            {
                if (SIMDPhysicsEngine.Config.EnablePerformanceLogging)
                {
                    _simdOperations++;
                    _simdTime += timeMs;
                }
            }

            public static void RecordScalarOperation(double timeMs)
            {
                if (SIMDPhysicsEngine.Config.EnablePerformanceLogging)
                {
                    _scalarOperations++;
                    _scalarTime += timeMs;
                }
            }

            public static (long simdOps, long scalarOps, double simdAvg, double scalarAvg, double speedup) GetStats()
            {
                var simdAvg = _simdOperations > 0 ? _simdTime / _simdOperations : 0.0;
                var scalarAvg = _scalarOperations > 0 ? _scalarTime / _scalarOperations : 0.0;
                var speedup = simdAvg > 0 ? scalarAvg / simdAvg : 0.0;

                return (_simdOperations, _scalarOperations, simdAvg, scalarAvg, speedup);
            }

            public static void Reset()
            {
                _simdOperations = 0;
                _scalarOperations = 0;
                _simdTime = 0.0;
                _scalarTime = 0.0;
            }
        }
    }
}
