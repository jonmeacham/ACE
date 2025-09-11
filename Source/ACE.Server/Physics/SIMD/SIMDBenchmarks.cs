using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using ACE.Server.Physics.Animation;
using ACE.Server.Physics.Collision;

namespace ACE.Server.Physics.SIMD
{
    /// <summary>
    /// Comprehensive benchmark suite for SIMD physics optimizations.
    /// Provides performance comparison between SIMD and scalar implementations.
    /// </summary>
    public static class SIMDBenchmarks
    {
        /// <summary>
        /// Runs all SIMD benchmarks and returns performance results
        /// </summary>
        public static BenchmarkResults RunFullBenchmarkSuite(int vectorCount = 10000, int iterations = 100)
        {
            Console.WriteLine($"🚀 Running SIMD Physics Benchmark Suite...");
            Console.WriteLine($"📊 Testing with {vectorCount} vectors, {iterations} iterations");
            
            var results = new BenchmarkResults();
            
            // Vector3 Operations Benchmarks
            results.Vector3LengthSquared = BenchmarkVector3LengthSquared(vectorCount, iterations);
            results.Vector3DotProduct = BenchmarkVector3DotProduct(vectorCount, iterations);
            results.Vector3Normalize = BenchmarkVector3Normalize(vectorCount, iterations);
            results.Vector3Transform = BenchmarkVector3Transform(vectorCount, iterations);
            results.Vector3Lerp = BenchmarkVector3Lerp(vectorCount, iterations);
            
            // AFrame Operations Benchmarks
            results.AFrameCombine = BenchmarkAFrameCombine(vectorCount / 10, iterations);
            results.AFrameInterpolate = BenchmarkAFrameInterpolate(vectorCount / 10, iterations);
            
            // BBox Operations Benchmarks
            results.BBoxCompute = BenchmarkBBoxCompute(vectorCount, iterations);
            results.BBoxIntersect = BenchmarkBBoxIntersect(vectorCount / 10, iterations);
            
            // Matrix4x4 Operations Benchmarks
            results.Matrix4x4Multiply = BenchmarkMatrix4x4Multiply(vectorCount / 100, iterations);
            results.Matrix4x4Transpose = BenchmarkMatrix4x4Transpose(vectorCount / 100, iterations);
            
            PrintResults(results);
            return results;
        }
        
        /// <summary>
        /// Runs a quick integration test to verify SIMD functionality
        /// </summary>
        public static bool RunIntegrationTest()
        {
            Console.WriteLine("🧪 Running SIMD Integration Tests...");
            
            try
            {
                // Test Vector3 extensions
                var vectors = GenerateTestVectors(100);
                var lengths = SIMDVector3Extensions.LengthSquaredSIMD(vectors);
                if (lengths.Length != vectors.Length)
                {
                    Console.WriteLine("❌ Vector3 LengthSquared test failed");
                    return false;
                }
                
                // Test AFrame operations
                var framesA = GenerateTestAFrames(50);
                var framesB = GenerateTestAFrames(50);
                var results = new AFrame[50];
                SIMDAFrame.CombineBatch(framesA, framesB, results);
                
                // Test BBox operations
                var vertices = GenerateTestVectors(200);
                SIMDBBox.ComputeBoundingBox(vertices, out var min, out var max);
                if (min == Vector3.Zero && max == Vector3.Zero)
                {
                    Console.WriteLine("❌ BBox computation test failed");
                    return false;
                }
                
                // Test Matrix operations
                var matrices1 = GenerateTestMatrices(20);
                var matrices2 = GenerateTestMatrices(20);
                var matrixResults = new Matrix4x4[20];
                SIMDMatrix4x4.MultiplyBatch(matrices1, matrices2, matrixResults);
                
                Console.WriteLine("✅ All integration tests passed!");
                return true;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"❌ Integration test failed: {ex.Message}");
                return false;
            }
        }
        
        #region Individual Benchmark Methods
        
        private static BenchmarkResult BenchmarkVector3LengthSquared(int count, int iterations)
        {
            var vectors = GenerateTestVectors(count);
            var sw = new Stopwatch();
            
            // SIMD test
            sw.Start();
            for (int i = 0; i < iterations; i++)
            {
                var results = SIMDVector3Extensions.LengthSquaredSIMD(vectors);
            }
            sw.Stop();
            var simdTime = sw.ElapsedMilliseconds;
            
            // Scalar test
            sw.Restart();
            for (int i = 0; i < iterations; i++)
            {
                var results = new float[vectors.Length];
                for (int j = 0; j < vectors.Length; j++)
                {
                    results[j] = vectors[j].LengthSquared();
                }
            }
            sw.Stop();
            var scalarTime = sw.ElapsedMilliseconds;
            
            return new BenchmarkResult("Vector3 LengthSquared", simdTime, scalarTime);
        }
        
        private static BenchmarkResult BenchmarkVector3DotProduct(int count, int iterations)
        {
            var vectorsA = GenerateTestVectors(count);
            var vectorsB = GenerateTestVectors(count);
            var results = new float[count];
            var sw = new Stopwatch();
            
            // SIMD test
            sw.Start();
            for (int i = 0; i < iterations; i++)
            {
                SIMDVector3Extensions.DotProductBatch(vectorsA, vectorsB, results);
            }
            sw.Stop();
            var simdTime = sw.ElapsedMilliseconds;
            
            // Scalar test
            sw.Restart();
            for (int i = 0; i < iterations; i++)
            {
                for (int j = 0; j < count; j++)
                {
                    results[j] = Vector3.Dot(vectorsA[j], vectorsB[j]);
                }
            }
            sw.Stop();
            var scalarTime = sw.ElapsedMilliseconds;
            
            return new BenchmarkResult("Vector3 DotProduct", simdTime, scalarTime);
        }
        
        private static BenchmarkResult BenchmarkVector3Normalize(int count, int iterations)
        {
            var vectors = GenerateTestVectors(count);
            var results = new Vector3[count];
            var sw = new Stopwatch();
            
            // SIMD test
            sw.Start();
            for (int i = 0; i < iterations; i++)
            {
                SIMDVector3Extensions.NormalizeBatch(vectors, results);
            }
            sw.Stop();
            var simdTime = sw.ElapsedMilliseconds;
            
            // Scalar test
            sw.Restart();
            for (int i = 0; i < iterations; i++)
            {
                for (int j = 0; j < count; j++)
                {
                    results[j] = Vector3.Normalize(vectors[j]);
                }
            }
            sw.Stop();
            var scalarTime = sw.ElapsedMilliseconds;
            
            return new BenchmarkResult("Vector3 Normalize", simdTime, scalarTime);
        }
        
        private static BenchmarkResult BenchmarkVector3Transform(int count, int iterations)
        {
            var vectors = GenerateTestVectors(count);
            var transform = Matrix4x4.CreateRotationY(0.5f);
            var results = new Vector3[count];
            var sw = new Stopwatch();
            
            // SIMD test
            sw.Start();
            for (int i = 0; i < iterations; i++)
            {
                SIMDVector3Extensions.TransformBatch(vectors, results, transform);
            }
            sw.Stop();
            var simdTime = sw.ElapsedMilliseconds;
            
            // Scalar test
            sw.Restart();
            for (int i = 0; i < iterations; i++)
            {
                for (int j = 0; j < count; j++)
                {
                    results[j] = Vector3.Transform(vectors[j], transform);
                }
            }
            sw.Stop();
            var scalarTime = sw.ElapsedMilliseconds;
            
            return new BenchmarkResult("Vector3 Transform", simdTime, scalarTime);
        }
        
        private static BenchmarkResult BenchmarkVector3Lerp(int count, int iterations)
        {
            var vectorsA = GenerateTestVectors(count);
            var vectorsB = GenerateTestVectors(count);
            var t = GenerateTestFloats(count);
            var results = new Vector3[count];
            var sw = new Stopwatch();
            
            // SIMD test
            sw.Start();
            for (int i = 0; i < iterations; i++)
            {
                SIMDVector3Extensions.LerpBatch(vectorsA, vectorsB, t, results);
            }
            sw.Stop();
            var simdTime = sw.ElapsedMilliseconds;
            
            // Scalar test
            sw.Restart();
            for (int i = 0; i < iterations; i++)
            {
                for (int j = 0; j < count; j++)
                {
                    results[j] = Vector3.Lerp(vectorsA[j], vectorsB[j], t[j]);
                }
            }
            sw.Stop();
            var scalarTime = sw.ElapsedMilliseconds;
            
            return new BenchmarkResult("Vector3 Lerp", simdTime, scalarTime);
        }
        
        private static BenchmarkResult BenchmarkAFrameCombine(int count, int iterations)
        {
            var framesA = GenerateTestAFrames(count);
            var framesB = GenerateTestAFrames(count);
            var results = new AFrame[count];
            var sw = new Stopwatch();
            
            // SIMD test
            sw.Start();
            for (int i = 0; i < iterations; i++)
            {
                SIMDAFrame.CombineBatch(framesA, framesB, results);
            }
            sw.Stop();
            var simdTime = sw.ElapsedMilliseconds;
            
            // Scalar test
            sw.Restart();
            for (int i = 0; i < iterations; i++)
            {
                for (int j = 0; j < count; j++)
                {
                    results[j] = AFrame.Combine(framesA[j], framesB[j]);
                }
            }
            sw.Stop();
            var scalarTime = sw.ElapsedMilliseconds;
            
            return new BenchmarkResult("AFrame Combine", simdTime, scalarTime);
        }
        
        private static BenchmarkResult BenchmarkAFrameInterpolate(int count, int iterations)
        {
            var framesA = GenerateTestAFrames(count);
            var framesB = GenerateTestAFrames(count);
            var t = GenerateTestFloats(count);
            var results = new AFrame[count];
            var sw = new Stopwatch();
            
            // SIMD test
            sw.Start();
            for (int i = 0; i < iterations; i++)
            {
                SIMDAFrame.InterpolateBatch(framesA, framesB, t, results);
            }
            sw.Stop();
            var simdTime = sw.ElapsedMilliseconds;
            
            // Scalar test (manual interpolation)
            sw.Restart();
            for (int i = 0; i < iterations; i++)
            {
                for (int j = 0; j < count; j++)
                {
                    var origin = Vector3.Lerp(framesA[j].Origin, framesB[j].Origin, t[j]);
                    var orientation = Quaternion.Lerp(framesA[j].Orientation, framesB[j].Orientation, t[j]);
                    results[j] = new AFrame(origin, orientation);
                }
            }
            sw.Stop();
            var scalarTime = sw.ElapsedMilliseconds;
            
            return new BenchmarkResult("AFrame Interpolate", simdTime, scalarTime);
        }
        
        private static BenchmarkResult BenchmarkBBoxCompute(int count, int iterations)
        {
            var vertices = GenerateTestVectors(count);
            var sw = new Stopwatch();
            
            // SIMD test
            sw.Start();
            for (int i = 0; i < iterations; i++)
            {
                SIMDBBox.ComputeBoundingBox(vertices, out var min, out var max);
            }
            sw.Stop();
            var simdTime = sw.ElapsedMilliseconds;
            
            // Scalar test
            sw.Restart();
            for (int i = 0; i < iterations; i++)
            {
                var min = vertices[0];
                var max = vertices[0];
                for (int j = 1; j < vertices.Length; j++)
                {
                    if (vertices[j].X < min.X) min.X = vertices[j].X;
                    if (vertices[j].Y < min.Y) min.Y = vertices[j].Y;
                    if (vertices[j].Z < min.Z) min.Z = vertices[j].Z;
                    if (vertices[j].X > max.X) max.X = vertices[j].X;
                    if (vertices[j].Y > max.Y) max.Y = vertices[j].Y;
                    if (vertices[j].Z > max.Z) max.Z = vertices[j].Z;
                }
            }
            sw.Stop();
            var scalarTime = sw.ElapsedMilliseconds;
            
            return new BenchmarkResult("BBox Compute", simdTime, scalarTime);
        }
        
        private static BenchmarkResult BenchmarkBBoxIntersect(int count, int iterations)
        {
            var boxesA = GenerateTestBBoxes(count);
            var boxesB = GenerateTestBBoxes(count);
            var results = new bool[count];
            var sw = new Stopwatch();
            
            // SIMD test
            sw.Start();
            for (int i = 0; i < iterations; i++)
            {
                SIMDBBox.IntersectBatch(boxesA, boxesB, results);
            }
            sw.Stop();
            var simdTime = sw.ElapsedMilliseconds;
            
            // Scalar test
            sw.Restart();
            for (int i = 0; i < iterations; i++)
            {
                for (int j = 0; j < count; j++)
                {
                    results[j] = (boxesA[j].Min.X <= boxesB[j].Max.X && boxesA[j].Max.X >= boxesB[j].Min.X) &&
                                 (boxesA[j].Min.Y <= boxesB[j].Max.Y && boxesA[j].Max.Y >= boxesB[j].Min.Y) &&
                                 (boxesA[j].Min.Z <= boxesB[j].Max.Z && boxesA[j].Max.Z >= boxesB[j].Min.Z);
                }
            }
            sw.Stop();
            var scalarTime = sw.ElapsedMilliseconds;
            
            return new BenchmarkResult("BBox Intersect", simdTime, scalarTime);
        }
        
        private static BenchmarkResult BenchmarkMatrix4x4Multiply(int count, int iterations)
        {
            var matricesA = GenerateTestMatrices(count);
            var matricesB = GenerateTestMatrices(count);
            var results = new Matrix4x4[count];
            var sw = new Stopwatch();
            
            // SIMD test
            sw.Start();
            for (int i = 0; i < iterations; i++)
            {
                SIMDMatrix4x4.MultiplyBatch(matricesA, matricesB, results);
            }
            sw.Stop();
            var simdTime = sw.ElapsedMilliseconds;
            
            // Scalar test
            sw.Restart();
            for (int i = 0; i < iterations; i++)
            {
                for (int j = 0; j < count; j++)
                {
                    results[j] = Matrix4x4.Multiply(matricesA[j], matricesB[j]);
                }
            }
            sw.Stop();
            var scalarTime = sw.ElapsedMilliseconds;
            
            return new BenchmarkResult("Matrix4x4 Multiply", simdTime, scalarTime);
        }
        
        private static BenchmarkResult BenchmarkMatrix4x4Transpose(int count, int iterations)
        {
            var matrices = GenerateTestMatrices(count);
            var results = new Matrix4x4[count];
            var sw = new Stopwatch();
            
            // SIMD test
            sw.Start();
            for (int i = 0; i < iterations; i++)
            {
                SIMDMatrix4x4.TransposeBatch(matrices, results);
            }
            sw.Stop();
            var simdTime = sw.ElapsedMilliseconds;
            
            // Scalar test
            sw.Restart();
            for (int i = 0; i < iterations; i++)
            {
                for (int j = 0; j < count; j++)
                {
                    results[j] = Matrix4x4.Transpose(matrices[j]);
                }
            }
            sw.Stop();
            var scalarTime = sw.ElapsedMilliseconds;
            
            return new BenchmarkResult("Matrix4x4 Transpose", simdTime, scalarTime);
        }
        
        #endregion
        
        #region Test Data Generation
        
        private static Vector3[] GenerateTestVectors(int count)
        {
            var random = new Random(42); // Fixed seed for consistent benchmarks
            var vectors = new Vector3[count];
            for (int i = 0; i < count; i++)
            {
                vectors[i] = new Vector3(
                    (float)random.NextDouble() * 100,
                    (float)random.NextDouble() * 100,
                    (float)random.NextDouble() * 100
                );
            }
            return vectors;
        }
        
        private static AFrame[] GenerateTestAFrames(int count)
        {
            var random = new Random(42);
            var frames = new AFrame[count];
            for (int i = 0; i < count; i++)
            {
                var origin = new Vector3(
                    (float)random.NextDouble() * 100,
                    (float)random.NextDouble() * 100,
                    (float)random.NextDouble() * 100
                );
                var orientation = Quaternion.CreateFromYawPitchRoll(
                    (float)random.NextDouble() * MathF.PI,
                    (float)random.NextDouble() * MathF.PI,
                    (float)random.NextDouble() * MathF.PI
                );
                frames[i] = new AFrame(origin, orientation);
            }
            return frames;
        }
        
        private static BBox[] GenerateTestBBoxes(int count)
        {
            var random = new Random(42);
            var boxes = new BBox[count];
            for (int i = 0; i < count; i++)
            {
                var center = new Vector3(
                    (float)random.NextDouble() * 100,
                    (float)random.NextDouble() * 100,
                    (float)random.NextDouble() * 100
                );
                var size = new Vector3(
                    (float)random.NextDouble() * 10 + 1,
                    (float)random.NextDouble() * 10 + 1,
                    (float)random.NextDouble() * 10 + 1
                );
                boxes[i] = new BBox()
                {
                    Min = center - size * 0.5f,
                    Max = center + size * 0.5f
                };
            }
            return boxes;
        }
        
        private static Matrix4x4[] GenerateTestMatrices(int count)
        {
            var random = new Random(42);
            var matrices = new Matrix4x4[count];
            for (int i = 0; i < count; i++)
            {
                var rotation = Quaternion.CreateFromYawPitchRoll(
                    (float)random.NextDouble() * MathF.PI,
                    (float)random.NextDouble() * MathF.PI,
                    (float)random.NextDouble() * MathF.PI
                );
                var translation = new Vector3(
                    (float)random.NextDouble() * 100,
                    (float)random.NextDouble() * 100,
                    (float)random.NextDouble() * 100
                );
                var scale = new Vector3(
                    (float)random.NextDouble() * 2 + 0.5f,
                    (float)random.NextDouble() * 2 + 0.5f,
                    (float)random.NextDouble() * 2 + 0.5f
                );
                
                matrices[i] = Matrix4x4.CreateScale(scale) * 
                             Matrix4x4.CreateFromQuaternion(rotation) * 
                             Matrix4x4.CreateTranslation(translation);
            }
            return matrices;
        }
        
        private static float[] GenerateTestFloats(int count)
        {
            var random = new Random(42);
            var floats = new float[count];
            for (int i = 0; i < count; i++)
            {
                floats[i] = (float)random.NextDouble();
            }
            return floats;
        }
        
        #endregion
        
        #region Results Display
        
        private static void PrintResults(BenchmarkResults results)
        {
            Console.WriteLine("\n📊 SIMD Physics Benchmark Results:");
            Console.WriteLine("=" + new string('=', 70));
            
            PrintResult(results.Vector3LengthSquared);
            PrintResult(results.Vector3DotProduct);
            PrintResult(results.Vector3Normalize);
            PrintResult(results.Vector3Transform);
            PrintResult(results.Vector3Lerp);
            PrintResult(results.AFrameCombine);
            PrintResult(results.AFrameInterpolate);
            PrintResult(results.BBoxCompute);
            PrintResult(results.BBoxIntersect);
            PrintResult(results.Matrix4x4Multiply);
            PrintResult(results.Matrix4x4Transpose);
            
            Console.WriteLine("=" + new string('=', 70));
            Console.WriteLine($"🏆 Average SIMD speedup: {results.AverageSpeedup:F2}x");
            Console.WriteLine($"⚡ Hardware acceleration: {(Vector.IsHardwareAccelerated ? "ENABLED" : "DISABLED")}");
            Console.WriteLine($"🔢 Vector<float>.Count: {Vector<float>.Count}");
        }
        
        private static void PrintResult(BenchmarkResult result)
        {
            var speedup = result.Speedup;
            var speedupColor = speedup > 1.5 ? "🟢" : speedup > 1.0 ? "🟡" : "🔴";
            Console.WriteLine($"{speedupColor} {result.Operation,-25} | SIMD: {result.SIMDTime,5}ms | Scalar: {result.ScalarTime,5}ms | Speedup: {speedup,5:F2}x");
        }
        
        #endregion
    }
    
    #region Data Structures
    
    public class BenchmarkResult
    {
        public string Operation { get; }
        public long SIMDTime { get; }
        public long ScalarTime { get; }
        public double Speedup => ScalarTime > 0 ? (double)ScalarTime / SIMDTime : 0;
        
        public BenchmarkResult(string operation, long simdTime, long scalarTime)
        {
            Operation = operation;
            SIMDTime = simdTime;
            ScalarTime = scalarTime;
        }
    }
    
    public class BenchmarkResults
    {
        public BenchmarkResult Vector3LengthSquared { get; set; }
        public BenchmarkResult Vector3DotProduct { get; set; }
        public BenchmarkResult Vector3Normalize { get; set; }
        public BenchmarkResult Vector3Transform { get; set; }
        public BenchmarkResult Vector3Lerp { get; set; }
        public BenchmarkResult AFrameCombine { get; set; }
        public BenchmarkResult AFrameInterpolate { get; set; }
        public BenchmarkResult BBoxCompute { get; set; }
        public BenchmarkResult BBoxIntersect { get; set; }
        public BenchmarkResult Matrix4x4Multiply { get; set; }
        public BenchmarkResult Matrix4x4Transpose { get; set; }
        
        public double AverageSpeedup
        {
            get
            {
                var results = new[] 
                {
                    Vector3LengthSquared, Vector3DotProduct, Vector3Normalize, Vector3Transform, Vector3Lerp,
                    AFrameCombine, AFrameInterpolate, BBoxCompute, BBoxIntersect, Matrix4x4Multiply, Matrix4x4Transpose
                };
                
                double totalSpeedup = 0;
                int count = 0;
                
                foreach (var result in results)
                {
                    if (result != null)
                    {
                        totalSpeedup += result.Speedup;
                        count++;
                    }
                }
                
                return count > 0 ? totalSpeedup / count : 0;
            }
        }
    }
    
    #endregion
}
