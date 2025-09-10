using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using ACE.Server.Physics;
using ACE.Server.Physics.Animation;
using ACE.Server.Physics.Common;

namespace ACE.Server.Physics.SIMD
{
    /// <summary>
    /// Simple integration test to validate SIMD physics optimizations are working
    /// </summary>
    public static class SIMDIntegrationTest
    {
        /// <summary>
        /// Performs a basic integration test of SIMD physics optimizations
        /// </summary>
        public static bool RunIntegrationTest()
        {
            try
            {
                Console.WriteLine("🧪 Running SIMD Physics Integration Test...");
                
                // Test 1: SIMD Configuration
                if (!TestSIMDConfiguration())
                {
                    Console.WriteLine("❌ SIMD Configuration test failed");
                    return false;
                }
                Console.WriteLine("✅ SIMD Configuration test passed");

                // Test 2: Vector Operations
                if (!TestSIMDVectorOperations())
                {
                    Console.WriteLine("❌ SIMD Vector Operations test failed");
                    return false;
                }
                Console.WriteLine("✅ SIMD Vector Operations test passed");

                // Test 3: Performance Monitor
                if (!TestPerformanceMonitoring())
                {
                    Console.WriteLine("❌ Performance Monitoring test failed");
                    return false;
                }
                Console.WriteLine("✅ Performance Monitoring test passed");

                // Test 4: Batch Processing Capability
                if (!TestBatchProcessingCapability())
                {
                    Console.WriteLine("❌ Batch Processing test failed");
                    return false;
                }
                Console.WriteLine("✅ Batch Processing test passed");

                Console.WriteLine("🎉 All SIMD Integration tests passed!");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"💥 SIMD Integration Test failed with exception: {ex.Message}");
                return false;
            }
        }

        private static bool TestSIMDConfiguration()
        {
            // Test that SIMD configuration is accessible and has reasonable defaults
            var config = SIMDPhysicsEngine.Config;
            
            return config.EnableSIMDOptimizations &&
                   config.MinObjectsForSIMD > 0 &&
                   config.MinObjectsForSIMD < 100; // Reasonable range
        }

        private static bool TestSIMDVectorOperations()
        {
            // Test basic SIMD vector operations
            var vectors = new Vector3[]
            {
                new Vector3(1, 2, 3),
                new Vector3(4, 5, 6),
                new Vector3(7, 8, 9),
                new Vector3(10, 11, 12)
            };

            var results = vectors.LengthSquaredSIMD();
            
            // Verify results are reasonable (non-zero, finite)
            foreach (var result in results)
            {
                if (result <= 0 || !float.IsFinite(result))
                    return false;
            }

            return results.Length == vectors.Length;
        }

        private static bool TestPerformanceMonitoring()
        {
            var monitor = PhysicsObjExtensions.PerformanceMonitor;
            
            // Record test operations
            monitor.RecordSIMDOperation(1.5);
            monitor.RecordScalarOperation(3.0);
            
            // Verify metrics are tracking
            return monitor.SIMDOperationCount > 0 && 
                   monitor.ScalarOperationCount > 0;
        }

        private static bool TestBatchProcessingCapability()
        {
            // Create mock physics objects for testing
            var physicsObjects = new List<PhysicsObj>();
            
            for (int i = 0; i < 5; i++)
            {
                var obj = new PhysicsObj();
                obj.Velocity = new Vector3(i, i + 1, i + 2);
                obj.Acceleration = new Vector3(0.1f, 0.1f, 0.1f);
                physicsObjects.Add(obj);
            }

            // Test that batch processing doesn't crash
            try
            {
                SIMDPhysicsEngine.UpdatePhysicsBatch(physicsObjects, 0.016f); // 16ms frame time
                return true;
            }
            catch (Exception)
            {
                return false;
            }
        }

        /// <summary>
        /// Runs a performance comparison between SIMD and scalar physics operations
        /// </summary>
        public static void RunPerformanceComparison(int iterations = 1000)
        {
            Console.WriteLine($"📊 Running SIMD Performance Comparison ({iterations} iterations)...");

            var random = new Random(42); // Fixed seed for reproducible results
            var physicsObjects = new List<PhysicsObj>();

            // Create test physics objects
            for (int i = 0; i < 50; i++)
            {
                var obj = new PhysicsObj();
                obj.Velocity = new Vector3(
                    (float)(random.NextDouble() - 0.5) * 10,
                    (float)(random.NextDouble() - 0.5) * 10,
                    (float)(random.NextDouble() - 0.5) * 10
                );
                obj.Acceleration = new Vector3(
                    (float)(random.NextDouble() - 0.5),
                    (float)(random.NextDouble() - 0.5),
                    (float)(random.NextDouble() - 0.5)
                );
                physicsObjects.Add(obj);
            }

            // Warm up
            for (int i = 0; i < 10; i++)
            {
                SIMDPhysicsEngine.UpdatePhysicsBatch(physicsObjects, 0.016f);
            }

            // Benchmark SIMD
            var sw = Stopwatch.StartNew();
            for (int i = 0; i < iterations; i++)
            {
                SIMDPhysicsEngine.UpdatePhysicsBatch(physicsObjects, 0.016f);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark Scalar (individual updates)
            sw.Restart();
            for (int i = 0; i < iterations; i++)
            {
                foreach (var obj in physicsObjects)
                {
                    // Simple scalar physics update
                    obj.Velocity += obj.Acceleration * 0.016f;
                }
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            var speedup = scalarTime / simdTime;

            Console.WriteLine($"⚡ SIMD Time: {simdTime:F2} ms");
            Console.WriteLine($"🐌 Scalar Time: {scalarTime:F2} ms");
            Console.WriteLine($"🚀 Speedup: {speedup:F2}x");

            if (speedup > 1.1)
                Console.WriteLine("🎯 SIMD optimizations are providing performance benefits!");
            else if (speedup > 0.9)
                Console.WriteLine("✅ SIMD optimizations are working without performance penalty");
            else
                Console.WriteLine("⚠️ SIMD optimizations may need tuning - scalar is faster");
        }
    }
}
