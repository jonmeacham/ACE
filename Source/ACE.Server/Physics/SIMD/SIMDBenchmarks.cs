using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.CompilerServices;
using ACE.Server.Physics.Animation;
using ACE.Server.Physics.Collision;
using ACE.Server.Physics.Extensions;

namespace ACE.Server.Physics.SIMD
{
    /// <summary>
    /// Benchmarking infrastructure for SIMD physics optimizations.
    /// Provides performance comparison between SIMD and scalar implementations.
    /// </summary>
    public static class SIMDBenchmarks
    {
        public struct BenchmarkResult
        {
            public string TestName;
            public double ScalarTimeMs;
            public double SIMDTimeMs;
            public double SpeedupRatio;
            public bool ValidationPassed;
            public string ErrorMessage;

            public override string ToString()
            {
                if (!ValidationPassed)
                    return $"{TestName}: VALIDATION FAILED - {ErrorMessage}";

                return $"{TestName}: Scalar={ScalarTimeMs:F3}ms, SIMD={SIMDTimeMs:F3}ms, Speedup={SpeedupRatio:F2}x";
            }
        }

        /// <summary>
        /// Runs comprehensive benchmarks for all SIMD optimizations
        /// </summary>
        public static List<BenchmarkResult> RunAllBenchmarks()
        {
            var results = new List<BenchmarkResult>();

            Console.WriteLine("=== ACE Physics SIMD Benchmark Suite ===");
            Console.WriteLine($"Hardware Acceleration: SSE={Vector128.IsHardwareAccelerated}, AVX={Vector256.IsHardwareAccelerated}");
            Console.WriteLine();

            // Vector3 Operations
            results.Add(BenchmarkVector3Transform());
            results.Add(BenchmarkVector3DotProduct());
            results.Add(BenchmarkVector3Normalize());
            results.Add(BenchmarkVector3Lerp());

            // BBox Operations  
            results.Add(BenchmarkBBoxComputation());
            results.Add(BenchmarkBBoxContains());
            results.Add(BenchmarkTransformedBBox());

            // AFrame Operations
            results.Add(BenchmarkAFrameCombine());
            results.Add(BenchmarkAFrameInterpolation());
            results.Add(BenchmarkAFrameGlobalToLocal());

            // Matrix Operations
            results.Add(BenchmarkMatrix4x4Multiply());
            results.Add(BenchmarkMatrix4x4Transpose());
            results.Add(BenchmarkMatrix4x4FromQuaternion());

            Console.WriteLine("\n=== Benchmark Summary ===");
            foreach (var result in results)
            {
                Console.WriteLine(result);
            }

            var avgSpeedup = 0.0;
            var validTests = 0;
            foreach (var result in results)
            {
                if (result.ValidationPassed)
                {
                    avgSpeedup += result.SpeedupRatio;
                    validTests++;
                }
            }

            if (validTests > 0)
            {
                avgSpeedup /= validTests;
                Console.WriteLine($"\nAverage Speedup: {avgSpeedup:F2}x across {validTests} tests");
            }

            return results;
        }

        #region Vector3 Benchmarks

        private static BenchmarkResult BenchmarkVector3Transform()
        {
            const int testSize = 10000;
            const int iterations = 1000;

            var input = GenerateRandomVector3Array(testSize);
            var transform = Matrix4x4.CreateFromYawPitchRoll(0.1f, 0.2f, 0.3f) * Matrix4x4.CreateTranslation(1, 2, 3);
            var scalarOutput = new Vector3[testSize];
            var simdOutput = new Vector3[testSize];

            // Warm up
            for (int i = 0; i < input.Length; i++)
                scalarOutput[i] = Vector3.Transform(input[i], transform);
            SIMDVector3Extensions.TransformBatch(input, simdOutput, transform);

            // Benchmark scalar version
            var sw = Stopwatch.StartNew();
            for (int iter = 0; iter < iterations; iter++)
            {
                for (int i = 0; i < input.Length; i++)
                    scalarOutput[i] = Vector3.Transform(input[i], transform);
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark SIMD version
            sw.Restart();
            for (int iter = 0; iter < iterations; iter++)
            {
                SIMDVector3Extensions.TransformBatch(input, simdOutput, transform);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Validate results
            var validation = ValidateVector3Arrays(scalarOutput, simdOutput, 1e-5f);

            return new BenchmarkResult
            {
                TestName = "Vector3 Transform Batch",
                ScalarTimeMs = scalarTime,
                SIMDTimeMs = simdTime,
                SpeedupRatio = scalarTime / simdTime,
                ValidationPassed = validation.isValid,
                ErrorMessage = validation.error
            };
        }

        private static BenchmarkResult BenchmarkVector3DotProduct()
        {
            const int testSize = 10000;
            const int iterations = 1000;

            var vectorsA = GenerateRandomVector3Array(testSize);
            var vectorsB = GenerateRandomVector3Array(testSize);
            var scalarResults = new float[testSize];
            var simdResults = new float[testSize];

            // Benchmark scalar version
            var sw = Stopwatch.StartNew();
            for (int iter = 0; iter < iterations; iter++)
            {
                for (int i = 0; i < testSize; i++)
                    scalarResults[i] = Vector3.Dot(vectorsA[i], vectorsB[i]);
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark SIMD version
            sw.Restart();
            for (int iter = 0; iter < iterations; iter++)
            {
                SIMDVector3Extensions.DotProductBatch(vectorsA, vectorsB, simdResults);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Validate results
            var validation = ValidateFloatArrays(scalarResults, simdResults, 1e-5f);

            return new BenchmarkResult
            {
                TestName = "Vector3 Dot Product Batch",
                ScalarTimeMs = scalarTime,
                SIMDTimeMs = simdTime,
                SpeedupRatio = scalarTime / simdTime,
                ValidationPassed = validation.isValid,
                ErrorMessage = validation.error
            };
        }

        private static BenchmarkResult BenchmarkVector3Normalize()
        {
            const int testSize = 10000;
            const int iterations = 500;

            var input = GenerateRandomVector3Array(testSize);
            var scalarOutput = new Vector3[testSize];
            var simdOutput = new Vector3[testSize];

            // Benchmark scalar version
            var sw = Stopwatch.StartNew();
            for (int iter = 0; iter < iterations; iter++)
            {
                for (int i = 0; i < input.Length; i++)
                    scalarOutput[i] = Vector3.Normalize(input[i]);
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark SIMD version
            sw.Restart();
            for (int iter = 0; iter < iterations; iter++)
            {
                SIMDVector3Extensions.NormalizeBatch(input, simdOutput);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Validate results
            var validation = ValidateVector3Arrays(scalarOutput, simdOutput, 1e-4f);

            return new BenchmarkResult
            {
                TestName = "Vector3 Normalize Batch",
                ScalarTimeMs = scalarTime,
                SIMDTimeMs = simdTime,
                SpeedupRatio = scalarTime / simdTime,
                ValidationPassed = validation.isValid,
                ErrorMessage = validation.error
            };
        }

        private static BenchmarkResult BenchmarkVector3Lerp()
        {
            const int testSize = 10000;
            const int iterations = 1000;

            var from = GenerateRandomVector3Array(testSize);
            var to = GenerateRandomVector3Array(testSize);
            var t = GenerateRandomFloatArray(testSize, 0.0f, 1.0f);
            var scalarResults = new Vector3[testSize];
            var simdResults = new Vector3[testSize];

            // Benchmark scalar version
            var sw = Stopwatch.StartNew();
            for (int iter = 0; iter < iterations; iter++)
            {
                for (int i = 0; i < testSize; i++)
                    scalarResults[i] = Vector3.Lerp(from[i], to[i], t[i]);
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark SIMD version
            sw.Restart();
            for (int iter = 0; iter < iterations; iter++)
            {
                SIMDVector3Extensions.LerpBatch(from, to, t, simdResults);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Validate results
            var validation = ValidateVector3Arrays(scalarResults, simdResults, 1e-5f);

            return new BenchmarkResult
            {
                TestName = "Vector3 Lerp Batch",
                ScalarTimeMs = scalarTime,
                SIMDTimeMs = simdTime,
                SpeedupRatio = scalarTime / simdTime,
                ValidationPassed = validation.isValid,
                ErrorMessage = validation.error
            };
        }

        #endregion

        #region BBox Benchmarks

        private static BenchmarkResult BenchmarkBBoxComputation()
        {
            const int testSize = 10000;
            const int iterations = 500;

            var vertices = GenerateRandomVector3Array(testSize);

            // Benchmark scalar version
            var sw = Stopwatch.StartNew();
            Vector3 scalarMin = Vector3.Zero, scalarMax = Vector3.Zero;
            for (int iter = 0; iter < iterations; iter++)
            {
                scalarMin = scalarMax = vertices[0];
                for (int i = 1; i < vertices.Length; i++)
                {
                    scalarMin = Vector3.Min(scalarMin, vertices[i]);
                    scalarMax = Vector3.Max(scalarMax, vertices[i]);
                }
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark SIMD version
            sw.Restart();
            Vector3 simdMin = Vector3.Zero, simdMax = Vector3.Zero;
            for (int iter = 0; iter < iterations; iter++)
            {
                SIMDBBox.ComputeBoundingBox(vertices, out simdMin, out simdMax);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Validate results
            var validation = ValidateVector3(scalarMin, simdMin, 1e-5f) && ValidateVector3(scalarMax, simdMax, 1e-5f);

            return new BenchmarkResult
            {
                TestName = "BBox Computation",
                ScalarTimeMs = scalarTime,
                SIMDTimeMs = simdTime,
                SpeedupRatio = scalarTime / simdTime,
                ValidationPassed = validation,
                ErrorMessage = validation ? "" : "Min/Max values don't match"
            };
        }

        private static BenchmarkResult BenchmarkBBoxContains()
        {
            const int testSize = 10000;
            const int iterations = 1000;

            var points = GenerateRandomVector3Array(testSize);
            var mins = GenerateRandomVector3Array(testSize);
            var maxs = new Vector3[testSize];
            for (int i = 0; i < testSize; i++)
                maxs[i] = mins[i] + new Vector3(1, 1, 1); // Ensure max > min

            var scalarResults = new bool[testSize];
            var simdResults = new bool[testSize];

            // Benchmark scalar version
            var sw = Stopwatch.StartNew();
            for (int iter = 0; iter < iterations; iter++)
            {
                for (int i = 0; i < testSize; i++)
                {
                    var point = points[i];
                    var min = mins[i];
                    var max = maxs[i];
                    scalarResults[i] = (point.X >= min.X && point.X <= max.X) &&
                                      (point.Y >= min.Y && point.Y <= max.Y) &&
                                      (point.Z >= min.Z && point.Z <= max.Z);
                }
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark SIMD version
            sw.Restart();
            for (int iter = 0; iter < iterations; iter++)
            {
                SIMDBBox.ContainsBatch(points, mins, maxs, simdResults);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Validate results
            var validation = ValidateBoolArrays(scalarResults, simdResults);

            return new BenchmarkResult
            {
                TestName = "BBox Contains Batch",
                ScalarTimeMs = scalarTime,
                SIMDTimeMs = simdTime,
                SpeedupRatio = scalarTime / simdTime,
                ValidationPassed = validation.isValid,
                ErrorMessage = validation.error
            };
        }

        private static BenchmarkResult BenchmarkTransformedBBox()
        {
            const int testSize = 1000;
            const int iterations = 100;

            var vertices = GenerateRandomVector3Array(testSize);
            var transform = Matrix4x4.CreateFromYawPitchRoll(0.1f, 0.2f, 0.3f) * Matrix4x4.CreateTranslation(1, 2, 3);

            // Benchmark scalar version
            var sw = Stopwatch.StartNew();
            Vector3 scalarMin = Vector3.Zero, scalarMax = Vector3.Zero;
            for (int iter = 0; iter < iterations; iter++)
            {
                var first = Vector3.Transform(vertices[0], transform);
                scalarMin = scalarMax = first;
                for (int i = 1; i < vertices.Length; i++)
                {
                    var transformed = Vector3.Transform(vertices[i], transform);
                    scalarMin = Vector3.Min(scalarMin, transformed);
                    scalarMax = Vector3.Max(scalarMax, transformed);
                }
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark SIMD version
            sw.Restart();
            Vector3 simdMin = Vector3.Zero, simdMax = Vector3.Zero;
            for (int iter = 0; iter < iterations; iter++)
            {
                SIMDBBox.ComputeTransformedBoundingBox(vertices, transform, out simdMin, out simdMax);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Validate results
            var validation = ValidateVector3(scalarMin, simdMin, 1e-4f) && ValidateVector3(scalarMax, simdMax, 1e-4f);

            return new BenchmarkResult
            {
                TestName = "Transformed BBox",
                ScalarTimeMs = scalarTime,
                SIMDTimeMs = simdTime,
                SpeedupRatio = scalarTime / simdTime,
                ValidationPassed = validation,
                ErrorMessage = validation ? "" : "Transformed bounding box values don't match"
            };
        }

        #endregion

        #region AFrame Benchmarks

        private static BenchmarkResult BenchmarkAFrameCombine()
        {
            const int testSize = 5000;
            const int iterations = 1000;

            var framesA = GenerateRandomAFrameArray(testSize);
            var framesB = GenerateRandomAFrameArray(testSize);
            var scalarResults = new AFrame[testSize];
            var simdResults = new AFrame[testSize];

            // Benchmark scalar version
            var sw = Stopwatch.StartNew();
            for (int iter = 0; iter < iterations; iter++)
            {
                for (int i = 0; i < testSize; i++)
                    scalarResults[i] = AFrame.Combine(framesA[i], framesB[i]);
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark SIMD version
            sw.Restart();
            for (int iter = 0; iter < iterations; iter++)
            {
                SIMDAFrame.CombineBatch(framesA, framesB, simdResults);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Validate results
            var validation = ValidateAFrameArrays(scalarResults, simdResults, 1e-4f);

            return new BenchmarkResult
            {
                TestName = "AFrame Combine Batch",
                ScalarTimeMs = scalarTime,
                SIMDTimeMs = simdTime,
                SpeedupRatio = scalarTime / simdTime,
                ValidationPassed = validation.isValid,
                ErrorMessage = validation.error
            };
        }

        private static BenchmarkResult BenchmarkAFrameInterpolation()
        {
            const int testSize = 5000;
            const int iterations = 500;

            var from = GenerateRandomAFrameArray(testSize);
            var to = GenerateRandomAFrameArray(testSize);
            var t = GenerateRandomFloatArray(testSize, 0.0f, 1.0f);
            var scalarResults = new AFrame[testSize];
            var simdResults = new AFrame[testSize];

            // Benchmark scalar version
            var sw = Stopwatch.StartNew();
            for (int iter = 0; iter < iterations; iter++)
            {
                for (int i = 0; i < testSize; i++)
                {
                    var result = new AFrame();
                    result.InterpolateOrigin(from[i], to[i], t[i]);
                    result.InterpolateRotation(from[i], to[i], t[i]);
                    scalarResults[i] = result;
                }
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark SIMD version
            sw.Restart();
            for (int iter = 0; iter < iterations; iter++)
            {
                SIMDAFrame.InterpolateBatch(from, to, t, simdResults);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Validate results
            var validation = ValidateAFrameArrays(scalarResults, simdResults, 1e-3f); // Slightly relaxed due to quaternion slerp

            return new BenchmarkResult
            {
                TestName = "AFrame Interpolation Batch",
                ScalarTimeMs = scalarTime,
                SIMDTimeMs = simdTime,
                SpeedupRatio = scalarTime / simdTime,
                ValidationPassed = validation.isValid,
                ErrorMessage = validation.error
            };
        }

        private static BenchmarkResult BenchmarkAFrameGlobalToLocal()
        {
            const int testSize = 5000;
            const int iterations = 1000;

            var frames = GenerateRandomAFrameArray(testSize);
            var globalPoints = GenerateRandomVector3Array(testSize);
            var scalarResults = new Vector3[testSize];
            var simdResults = new Vector3[testSize];

            // Benchmark scalar version
            var sw = Stopwatch.StartNew();
            for (int iter = 0; iter < iterations; iter++)
            {
                for (int i = 0; i < testSize; i++)
                    scalarResults[i] = frames[i].GlobalToLocal(globalPoints[i]);
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark SIMD version
            sw.Restart();
            for (int iter = 0; iter < iterations; iter++)
            {
                SIMDAFrame.GlobalToLocalBatch(frames, globalPoints, simdResults);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Validate results
            var validation = ValidateVector3Arrays(scalarResults, simdResults, 1e-4f);

            return new BenchmarkResult
            {
                TestName = "AFrame Global To Local Batch",
                ScalarTimeMs = scalarTime,
                SIMDTimeMs = simdTime,
                SpeedupRatio = scalarTime / simdTime,
                ValidationPassed = validation.isValid,
                ErrorMessage = validation.error
            };
        }

        #endregion

        #region Matrix4x4 Benchmarks

        private static BenchmarkResult BenchmarkMatrix4x4Multiply()
        {
            const int testSize = 5000;
            const int iterations = 500;

            var matricesA = GenerateRandomMatrix4x4Array(testSize);
            var matricesB = GenerateRandomMatrix4x4Array(testSize);
            var scalarResults = new Matrix4x4[testSize];
            var simdResults = new Matrix4x4[testSize];

            // Benchmark scalar version
            var sw = Stopwatch.StartNew();
            for (int iter = 0; iter < iterations; iter++)
            {
                for (int i = 0; i < testSize; i++)
                    scalarResults[i] = Matrix4x4.Multiply(matricesA[i], matricesB[i]);
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark SIMD version
            sw.Restart();
            for (int iter = 0; iter < iterations; iter++)
            {
                SIMDMatrix4x4.MultiplyBatch(matricesA, matricesB, simdResults);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Validate results
            var validation = ValidateMatrix4x4Arrays(scalarResults, simdResults, 1e-4f);

            return new BenchmarkResult
            {
                TestName = "Matrix4x4 Multiply Batch",
                ScalarTimeMs = scalarTime,
                SIMDTimeMs = simdTime,
                SpeedupRatio = scalarTime / simdTime,
                ValidationPassed = validation.isValid,
                ErrorMessage = validation.error
            };
        }

        private static BenchmarkResult BenchmarkMatrix4x4Transpose()
        {
            const int testSize = 10000;
            const int iterations = 1000;

            var matrices = GenerateRandomMatrix4x4Array(testSize);
            var scalarResults = new Matrix4x4[testSize];
            var simdResults = new Matrix4x4[testSize];

            // Benchmark scalar version
            var sw = Stopwatch.StartNew();
            for (int iter = 0; iter < iterations; iter++)
            {
                for (int i = 0; i < testSize; i++)
                    scalarResults[i] = Matrix4x4.Transpose(matrices[i]);
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark SIMD version
            sw.Restart();
            for (int iter = 0; iter < iterations; iter++)
            {
                SIMDMatrix4x4.TransposeBatch(matrices, simdResults);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Validate results
            var validation = ValidateMatrix4x4Arrays(scalarResults, simdResults, 1e-6f);

            return new BenchmarkResult
            {
                TestName = "Matrix4x4 Transpose Batch",
                ScalarTimeMs = scalarTime,
                SIMDTimeMs = simdTime,
                SpeedupRatio = scalarTime / simdTime,
                ValidationPassed = validation.isValid,
                ErrorMessage = validation.error
            };
        }

        private static BenchmarkResult BenchmarkMatrix4x4FromQuaternion()
        {
            const int testSize = 10000;
            const int iterations = 1000;

            var quaternions = GenerateRandomQuaternionArray(testSize);
            var scalarResults = new Matrix4x4[testSize];
            var simdResults = new Matrix4x4[testSize];

            // Benchmark scalar version
            var sw = Stopwatch.StartNew();
            for (int iter = 0; iter < iterations; iter++)
            {
                for (int i = 0; i < testSize; i++)
                    scalarResults[i] = Matrix4x4.CreateFromQuaternion(quaternions[i]);
            }
            sw.Stop();
            var scalarTime = sw.Elapsed.TotalMilliseconds;

            // Benchmark SIMD version
            sw.Restart();
            for (int iter = 0; iter < iterations; iter++)
            {
                SIMDMatrix4x4.CreateFromQuaternionBatch(quaternions, simdResults);
            }
            sw.Stop();
            var simdTime = sw.Elapsed.TotalMilliseconds;

            // Validate results
            var validation = ValidateMatrix4x4Arrays(scalarResults, simdResults, 1e-5f);

            return new BenchmarkResult
            {
                TestName = "Matrix4x4 From Quaternion Batch",
                ScalarTimeMs = scalarTime,
                SIMDTimeMs = simdTime,
                SpeedupRatio = scalarTime / simdTime,
                ValidationPassed = validation.isValid,
                ErrorMessage = validation.error
            };
        }

        #endregion

        #region Helper Methods

        private static Vector3[] GenerateRandomVector3Array(int size)
        {
            var random = new Random(12345); // Fixed seed for reproducibility
            var array = new Vector3[size];
            for (int i = 0; i < size; i++)
            {
                array[i] = new Vector3(
                    (float)(random.NextDouble() * 200 - 100),
                    (float)(random.NextDouble() * 200 - 100),
                    (float)(random.NextDouble() * 200 - 100)
                );
            }
            return array;
        }

        private static float[] GenerateRandomFloatArray(int size, float min = -100f, float max = 100f)
        {
            var random = new Random(12345);
            var array = new float[size];
            for (int i = 0; i < size; i++)
            {
                array[i] = (float)(random.NextDouble() * (max - min) + min);
            }
            return array;
        }

        private static AFrame[] GenerateRandomAFrameArray(int size)
        {
            var random = new Random(12345);
            var array = new AFrame[size];
            for (int i = 0; i < size; i++)
            {
                var origin = new Vector3(
                    (float)(random.NextDouble() * 200 - 100),
                    (float)(random.NextDouble() * 200 - 100),
                    (float)(random.NextDouble() * 200 - 100)
                );
                var orientation = Quaternion.Normalize(new Quaternion(
                    (float)(random.NextDouble() * 2 - 1),
                    (float)(random.NextDouble() * 2 - 1),
                    (float)(random.NextDouble() * 2 - 1),
                    (float)(random.NextDouble() * 2 - 1)
                ));
                array[i] = new AFrame(origin, orientation);
            }
            return array;
        }

        private static Matrix4x4[] GenerateRandomMatrix4x4Array(int size)
        {
            var random = new Random(12345);
            var array = new Matrix4x4[size];
            for (int i = 0; i < size; i++)
            {
                // Create reasonable transformation matrices
                var translation = new Vector3(
                    (float)(random.NextDouble() * 20 - 10),
                    (float)(random.NextDouble() * 20 - 10),
                    (float)(random.NextDouble() * 20 - 10)
                );
                var rotation = Quaternion.Normalize(new Quaternion(
                    (float)(random.NextDouble() * 2 - 1),
                    (float)(random.NextDouble() * 2 - 1),
                    (float)(random.NextDouble() * 2 - 1),
                    (float)(random.NextDouble() * 2 - 1)
                ));
                var scale = new Vector3(
                    (float)(random.NextDouble() * 2 + 0.5),
                    (float)(random.NextDouble() * 2 + 0.5),
                    (float)(random.NextDouble() * 2 + 0.5)
                );

                array[i] = Matrix4x4.CreateScale(scale) * Matrix4x4.CreateFromQuaternion(rotation) * Matrix4x4.CreateTranslation(translation);
            }
            return array;
        }

        private static Quaternion[] GenerateRandomQuaternionArray(int size)
        {
            var random = new Random(12345);
            var array = new Quaternion[size];
            for (int i = 0; i < size; i++)
            {
                array[i] = Quaternion.Normalize(new Quaternion(
                    (float)(random.NextDouble() * 2 - 1),
                    (float)(random.NextDouble() * 2 - 1),
                    (float)(random.NextDouble() * 2 - 1),
                    (float)(random.NextDouble() * 2 - 1)
                ));
            }
            return array;
        }

        private static (bool isValid, string error) ValidateVector3Arrays(Vector3[] expected, Vector3[] actual, float tolerance)
        {
            if (expected.Length != actual.Length)
                return (false, $"Array lengths don't match: {expected.Length} vs {actual.Length}");

            for (int i = 0; i < expected.Length; i++)
            {
                if (!ValidateVector3(expected[i], actual[i], tolerance))
                    return (false, $"Mismatch at index {i}: {expected[i]} vs {actual[i]}");
            }
            return (true, "");
        }

        private static bool ValidateVector3(Vector3 expected, Vector3 actual, float tolerance)
        {
            return Math.Abs(expected.X - actual.X) <= tolerance &&
                   Math.Abs(expected.Y - actual.Y) <= tolerance &&
                   Math.Abs(expected.Z - actual.Z) <= tolerance;
        }

        private static (bool isValid, string error) ValidateFloatArrays(float[] expected, float[] actual, float tolerance)
        {
            if (expected.Length != actual.Length)
                return (false, $"Array lengths don't match: {expected.Length} vs {actual.Length}");

            for (int i = 0; i < expected.Length; i++)
            {
                if (Math.Abs(expected[i] - actual[i]) > tolerance)
                    return (false, $"Mismatch at index {i}: {expected[i]} vs {actual[i]}");
            }
            return (true, "");
        }

        private static (bool isValid, string error) ValidateBoolArrays(bool[] expected, bool[] actual)
        {
            if (expected.Length != actual.Length)
                return (false, $"Array lengths don't match: {expected.Length} vs {actual.Length}");

            for (int i = 0; i < expected.Length; i++)
            {
                if (expected[i] != actual[i])
                    return (false, $"Mismatch at index {i}: {expected[i]} vs {actual[i]}");
            }
            return (true, "");
        }

        private static (bool isValid, string error) ValidateAFrameArrays(AFrame[] expected, AFrame[] actual, float tolerance)
        {
            if (expected.Length != actual.Length)
                return (false, $"Array lengths don't match: {expected.Length} vs {actual.Length}");

            for (int i = 0; i < expected.Length; i++)
            {
                if (!ValidateVector3(expected[i].Origin, actual[i].Origin, tolerance))
                    return (false, $"Origin mismatch at index {i}: {expected[i].Origin} vs {actual[i].Origin}");

                // Quaternion validation is more complex due to double-cover property
                var dot = Math.Abs(Quaternion.Dot(expected[i].Orientation, actual[i].Orientation));
                if (dot < 1.0f - tolerance) // Allow for q and -q being equivalent
                    return (false, $"Orientation mismatch at index {i}: {expected[i].Orientation} vs {actual[i].Orientation}");
            }
            return (true, "");
        }

        private static (bool isValid, string error) ValidateMatrix4x4Arrays(Matrix4x4[] expected, Matrix4x4[] actual, float tolerance)
        {
            if (expected.Length != actual.Length)
                return (false, $"Array lengths don't match: {expected.Length} vs {actual.Length}");

            for (int i = 0; i < expected.Length; i++)
            {
                if (!ValidateMatrix4x4(expected[i], actual[i], tolerance))
                    return (false, $"Matrix mismatch at index {i}");
            }
            return (true, "");
        }

        private static bool ValidateMatrix4x4(Matrix4x4 expected, Matrix4x4 actual, float tolerance)
        {
            return Math.Abs(expected.M11 - actual.M11) <= tolerance &&
                   Math.Abs(expected.M12 - actual.M12) <= tolerance &&
                   Math.Abs(expected.M13 - actual.M13) <= tolerance &&
                   Math.Abs(expected.M14 - actual.M14) <= tolerance &&
                   Math.Abs(expected.M21 - actual.M21) <= tolerance &&
                   Math.Abs(expected.M22 - actual.M22) <= tolerance &&
                   Math.Abs(expected.M23 - actual.M23) <= tolerance &&
                   Math.Abs(expected.M24 - actual.M24) <= tolerance &&
                   Math.Abs(expected.M31 - actual.M31) <= tolerance &&
                   Math.Abs(expected.M32 - actual.M32) <= tolerance &&
                   Math.Abs(expected.M33 - actual.M33) <= tolerance &&
                   Math.Abs(expected.M34 - actual.M34) <= tolerance &&
                   Math.Abs(expected.M41 - actual.M41) <= tolerance &&
                   Math.Abs(expected.M42 - actual.M42) <= tolerance &&
                   Math.Abs(expected.M43 - actual.M43) <= tolerance &&
                   Math.Abs(expected.M44 - actual.M44) <= tolerance;
        }

        #endregion
    }
}
