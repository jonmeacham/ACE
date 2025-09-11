using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using ACE.Server.Physics.Common;

namespace ACE.Server.Physics.SIMD
{
    /// <summary>
    /// SIMD-optimized Vector3 operations for high-performance physics calculations.
    /// Provides significant speedups for batch vector operations common in game physics.
    /// </summary>
    public static unsafe class SIMDVector3Extensions
    {
        /// <summary>
        /// Computes length squared for multiple Vector3s simultaneously using SIMD
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float[] LengthSquaredSIMD(this Vector3[] vectors)
        {
            if (vectors == null || vectors.Length == 0)
                return Array.Empty<float>();

            var results = new float[vectors.Length];
            
            if (Vector.IsHardwareAccelerated && vectors.Length >= Vector<float>.Count)
            {
                LengthSquaredBatchVectorized(vectors, results);
            }
            else
            {
                LengthSquaredBatchScalar(vectors, results);
            }
            
            return results;
        }

        /// <summary>
        /// Transforms multiple Vector3 points using the same Matrix4x4 transformation
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformBatch(ReadOnlySpan<Vector3> input, Span<Vector3> output, Matrix4x4 transform)
        {
            if (input.Length != output.Length)
                throw new ArgumentException("Input and output spans must have the same length");

            if (Avx.IsSupported && input.Length >= 8)
            {
                TransformBatchAVX(input, output, transform);
            }
            else if (input.Length >= 4)
            {
                TransformBatchSSE(input, output, transform);
            }
            else
            {
                TransformBatchScalar(input, output, transform);
            }
        }

        /// <summary>
        /// Computes dot products for multiple Vector3 pairs simultaneously
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void DotProductBatch(ReadOnlySpan<Vector3> vectorsA, ReadOnlySpan<Vector3> vectorsB, Span<float> results)
        {
            if (vectorsA.Length != vectorsB.Length || vectorsA.Length != results.Length)
                throw new ArgumentException("All spans must have the same length");

            if (Vector.IsHardwareAccelerated && vectorsA.Length >= Vector<float>.Count)
            {
                DotProductBatchVectorized(vectorsA, vectorsB, results);
            }
            else
            {
                DotProductBatchScalar(vectorsA, vectorsB, results);
            }
        }

        /// <summary>
        /// Normalizes multiple Vector3s simultaneously with SIMD optimizations
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void NormalizeBatch(ReadOnlySpan<Vector3> input, Span<Vector3> output)
        {
            if (input.Length != output.Length)
                throw new ArgumentException("Input and output spans must have the same length");

            if (Vector.IsHardwareAccelerated && input.Length >= Vector<float>.Count)
            {
                NormalizeBatchVectorized(input, output);
            }
            else
            {
                NormalizeBatchScalar(input, output);
            }
        }

        #region Vectorized Implementations

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void LengthSquaredBatchVectorized(Vector3[] vectors, float[] results)
        {
            int vectorSize = Vector<float>.Count;
            int i = 0;

            // Process vectorizable chunks
            for (; i <= vectors.Length - vectorSize; i += vectorSize)
            {
                // Extract components into separate arrays for SIMD processing
                Span<float> xValues = stackalloc float[vectorSize];
                Span<float> yValues = stackalloc float[vectorSize];
                Span<float> zValues = stackalloc float[vectorSize];

                for (int j = 0; j < vectorSize; j++)
                {
                    var vec = vectors[i + j];
                    xValues[j] = vec.X;
                    yValues[j] = vec.Y;
                    zValues[j] = vec.Z;
                }

                // Create vectors from components
                var xVec = new Vector<float>(xValues);
                var yVec = new Vector<float>(yValues);
                var zVec = new Vector<float>(zValues);

                // Compute x*x + y*y + z*z
                var result = xVec * xVec + yVec * yVec + zVec * zVec;

                // Store results
                result.CopyTo(results.AsSpan(i, vectorSize));
            }

            // Handle remaining elements with scalar operations
            for (; i < vectors.Length; i++)
            {
                results[i] = vectors[i].LengthSquared();
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void DotProductBatchVectorized(ReadOnlySpan<Vector3> vectorsA, ReadOnlySpan<Vector3> vectorsB, Span<float> results)
        {
            int vectorSize = Vector<float>.Count;
            int i = 0;

            // Process vectorizable chunks
            for (; i <= vectorsA.Length - vectorSize; i += vectorSize)
            {
                // Extract components for SIMD processing
                Span<float> axValues = stackalloc float[vectorSize];
                Span<float> ayValues = stackalloc float[vectorSize];
                Span<float> azValues = stackalloc float[vectorSize];
                Span<float> bxValues = stackalloc float[vectorSize];
                Span<float> byValues = stackalloc float[vectorSize];
                Span<float> bzValues = stackalloc float[vectorSize];

                for (int j = 0; j < vectorSize; j++)
                {
                    var a = vectorsA[i + j];
                    var b = vectorsB[i + j];
                    axValues[j] = a.X; ayValues[j] = a.Y; azValues[j] = a.Z;
                    bxValues[j] = b.X; byValues[j] = b.Y; bzValues[j] = b.Z;
                }

                // Create vectors and compute dot products
                var axVec = new Vector<float>(axValues);
                var ayVec = new Vector<float>(ayValues);
                var azVec = new Vector<float>(azValues);
                var bxVec = new Vector<float>(bxValues);
                var byVec = new Vector<float>(byValues);
                var bzVec = new Vector<float>(bzValues);

                var result = axVec * bxVec + ayVec * byVec + azVec * bzVec;
                result.CopyTo(results.Slice(i, vectorSize));
            }

            // Handle remaining elements
            for (; i < vectorsA.Length; i++)
            {
                results[i] = Vector3.Dot(vectorsA[i], vectorsB[i]);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void NormalizeBatchVectorized(ReadOnlySpan<Vector3> input, Span<Vector3> output)
        {
            int vectorSize = Vector<float>.Count;
            int i = 0;

            // Process vectorizable chunks
            for (; i <= input.Length - vectorSize; i += vectorSize)
            {
                // Extract components
                Span<float> xValues = stackalloc float[vectorSize];
                Span<float> yValues = stackalloc float[vectorSize];
                Span<float> zValues = stackalloc float[vectorSize];

                for (int j = 0; j < vectorSize; j++)
                {
                    var vec = input[i + j];
                    xValues[j] = vec.X;
                    yValues[j] = vec.Y;
                    zValues[j] = vec.Z;
                }

                var xVec = new Vector<float>(xValues);
                var yVec = new Vector<float>(yValues);
                var zVec = new Vector<float>(zValues);

                // Compute length squared
                var lengthSq = xVec * xVec + yVec * yVec + zVec * zVec;

                // Compute reciprocal square root (approximate)
                var rsqrt = Vector.SquareRoot(lengthSq);
                var safeRsqrt = Vector.ConditionalSelect(
                    Vector.GreaterThan(rsqrt, new Vector<float>(PhysicsGlobals.EPSILON)),
                    Vector.Divide(Vector<float>.One, rsqrt),
                    Vector<float>.Zero);

                // Normalize
                var normalizedX = xVec * safeRsqrt;
                var normalizedY = yVec * safeRsqrt;
                var normalizedZ = zVec * safeRsqrt;

                // Store results
                Span<float> resultX = stackalloc float[vectorSize];
                Span<float> resultY = stackalloc float[vectorSize];
                Span<float> resultZ = stackalloc float[vectorSize];

                normalizedX.CopyTo(resultX);
                normalizedY.CopyTo(resultY);
                normalizedZ.CopyTo(resultZ);

                for (int j = 0; j < vectorSize; j++)
                {
                    output[i + j] = new Vector3(resultX[j], resultY[j], resultZ[j]);
                }
            }

            // Handle remaining elements
            for (; i < input.Length; i++)
            {
                var length = input[i].Length();
                output[i] = length > PhysicsGlobals.EPSILON ? input[i] / length : Vector3.Zero;
            }
        }

        #endregion

        #region AVX/SSE Fallback Implementations

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformBatchAVX(ReadOnlySpan<Vector3> input, Span<Vector3> output, Matrix4x4 transform)
        {
            // Simplified AVX implementation - can be enhanced later
            for (int i = 0; i < input.Length; i++)
            {
                output[i] = Vector3.Transform(input[i], transform);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformBatchSSE(ReadOnlySpan<Vector3> input, Span<Vector3> output, Matrix4x4 transform)
        {
            // Simplified SSE implementation - can be enhanced later  
            for (int i = 0; i < input.Length; i++)
            {
                output[i] = Vector3.Transform(input[i], transform);
            }
        }

        #endregion

        #region Scalar Implementations

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void LengthSquaredBatchScalar(Vector3[] vectors, float[] results)
        {
            for (int i = 0; i < vectors.Length; i++)
            {
                results[i] = vectors[i].LengthSquared();
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void DotProductBatchScalar(ReadOnlySpan<Vector3> vectorsA, ReadOnlySpan<Vector3> vectorsB, Span<float> results)
        {
            for (int i = 0; i < vectorsA.Length; i++)
            {
                results[i] = Vector3.Dot(vectorsA[i], vectorsB[i]);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformBatchScalar(ReadOnlySpan<Vector3> input, Span<Vector3> output, Matrix4x4 transform)
        {
            for (int i = 0; i < input.Length; i++)
            {
                output[i] = Vector3.Transform(input[i], transform);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void NormalizeBatchScalar(ReadOnlySpan<Vector3> input, Span<Vector3> output)
        {
            for (int i = 0; i < input.Length; i++)
            {
                var length = input[i].Length();
                output[i] = length > PhysicsGlobals.EPSILON ? input[i] / length : Vector3.Zero;
            }
        }

        #endregion
    }
}
