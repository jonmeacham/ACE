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
        /// Transforms multiple Vector3 points using the same Matrix4x4 transformation.
        /// Uses SIMD to process 4 vectors simultaneously, providing ~4x speedup over scalar operations.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformBatch(ReadOnlySpan<Vector3> input, Span<Vector3> output, Matrix4x4 transform)
        {
            if (input.Length != output.Length)
                throw new ArgumentException("Input and output spans must have the same length");

            if (Vector256.IsHardwareAccelerated && input.Length >= 8)
            {
                TransformBatchAVX(input, output, transform);
            }
            else if (Vector128.IsHardwareAccelerated && input.Length >= 4)
            {
                TransformBatchSSE(input, output, transform);
            }
            else
            {
                TransformBatchScalar(input, output, transform);
            }
        }

        /// <summary>
        /// AVX2-optimized batch transformation (8 vectors per iteration)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformBatchAVX(ReadOnlySpan<Vector3> input, Span<Vector3> output, Matrix4x4 transform)
        {
            // Extract transformation matrix components for SIMD operations
            var m11 = Vector256.Create(transform.M11);
            var m12 = Vector256.Create(transform.M12);
            var m13 = Vector256.Create(transform.M13);
            var m14 = Vector256.Create(transform.M14);

            var m21 = Vector256.Create(transform.M21);
            var m22 = Vector256.Create(transform.M22);
            var m23 = Vector256.Create(transform.M23);
            var m24 = Vector256.Create(transform.M24);

            var m31 = Vector256.Create(transform.M31);
            var m32 = Vector256.Create(transform.M32);
            var m33 = Vector256.Create(transform.M33);
            var m34 = Vector256.Create(transform.M34);

            var m41 = Vector256.Create(transform.M41);
            var m42 = Vector256.Create(transform.M42);
            var m43 = Vector256.Create(transform.M43);
            var m44 = Vector256.Create(transform.M44);

            int i = 0;
            int vectorizedLength = input.Length & ~7; // Process in groups of 8

            fixed (Vector3* pInput = input)
            fixed (Vector3* pOutput = output)
            {
                for (; i < vectorizedLength; i += 8)
                {
                    // Load 8 Vector3s (24 floats) - we'll need to handle this carefully
                    // For simplicity, we'll process 8 X values, then 8 Y values, then 8 Z values
                    var inputPtr = pInput + i;
                    var outputPtr = pOutput + i;

                    // Extract X, Y, Z components into separate vectors
                    Vector256<float> x, y, z;
                    LoadVector3Components(inputPtr, out x, out y, out z);

                    // Transform: result = M * [x, y, z, 1]
                    var resultX = Fma.MultiplyAdd(m11, x, Fma.MultiplyAdd(m12, y, Fma.MultiplyAdd(m13, z, m14)));
                    var resultY = Fma.MultiplyAdd(m21, x, Fma.MultiplyAdd(m22, y, Fma.MultiplyAdd(m23, z, m24)));
                    var resultZ = Fma.MultiplyAdd(m31, x, Fma.MultiplyAdd(m32, y, Fma.MultiplyAdd(m33, z, m34)));

                    // Store results back
                    StoreVector3Components(outputPtr, resultX, resultY, resultZ);
                }
            }

            // Handle remaining elements with scalar operations
            for (; i < input.Length; i++)
            {
                output[i] = Vector3.Transform(input[i], transform);
            }
        }

        /// <summary>
        /// SSE-optimized batch transformation (4 vectors per iteration)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformBatchSSE(ReadOnlySpan<Vector3> input, Span<Vector3> output, Matrix4x4 transform)
        {
            // Extract transformation matrix components for SIMD operations
            var m11 = Vector128.Create(transform.M11);
            var m12 = Vector128.Create(transform.M12);
            var m13 = Vector128.Create(transform.M13);
            var m14 = Vector128.Create(transform.M14);

            var m21 = Vector128.Create(transform.M21);
            var m22 = Vector128.Create(transform.M22);
            var m23 = Vector128.Create(transform.M23);
            var m24 = Vector128.Create(transform.M24);

            var m31 = Vector128.Create(transform.M31);
            var m32 = Vector128.Create(transform.M32);
            var m33 = Vector128.Create(transform.M33);
            var m34 = Vector128.Create(transform.M34);

            int i = 0;
            int vectorizedLength = input.Length & ~3; // Process in groups of 4

            fixed (Vector3* pInput = input)
            fixed (Vector3* pOutput = output)
            {
                for (; i < vectorizedLength; i += 4)
                {
                    var inputPtr = pInput + i;
                    var outputPtr = pOutput + i;

                    // Extract X, Y, Z components into separate vectors
                    Vector128<float> x, y, z;
                    LoadVector3Components128(inputPtr, out x, out y, out z);

                    // Transform: result = M * [x, y, z, 1]
                    var resultX = Fma.MultiplyAdd(m11, x, Fma.MultiplyAdd(m12, y, Fma.MultiplyAdd(m13, z, m14)));
                    var resultY = Fma.MultiplyAdd(m21, x, Fma.MultiplyAdd(m22, y, Fma.MultiplyAdd(m23, z, m24)));
                    var resultZ = Fma.MultiplyAdd(m31, x, Fma.MultiplyAdd(m32, y, Fma.MultiplyAdd(m33, z, m34)));

                    // Store results back
                    StoreVector3Components128(outputPtr, resultX, resultY, resultZ);
                }
            }

            // Handle remaining elements with scalar operations
            for (; i < input.Length; i++)
            {
                output[i] = Vector3.Transform(input[i], transform);
            }
        }

        /// <summary>
        /// Scalar fallback transformation
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformBatchScalar(ReadOnlySpan<Vector3> input, Span<Vector3> output, Matrix4x4 transform)
        {
            for (int i = 0; i < input.Length; i++)
            {
                output[i] = Vector3.Transform(input[i], transform);
            }
        }

        /// <summary>
        /// Loads Vector3 components into separate AVX vectors for processing 8 vectors at once
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void LoadVector3Components(Vector3* input, out Vector256<float> x, out Vector256<float> y, out Vector256<float> z)
        {
            // Load 8 Vector3s in a stride pattern to separate X, Y, Z components
            var v0 = input[0]; var v1 = input[1]; var v2 = input[2]; var v3 = input[3];
            var v4 = input[4]; var v5 = input[5]; var v6 = input[6]; var v7 = input[7];

            x = Vector256.Create(v0.X, v1.X, v2.X, v3.X, v4.X, v5.X, v6.X, v7.X);
            y = Vector256.Create(v0.Y, v1.Y, v2.Y, v3.Y, v4.Y, v5.Y, v6.Y, v7.Y);
            z = Vector256.Create(v0.Z, v1.Z, v2.Z, v3.Z, v4.Z, v5.Z, v6.Z, v7.Z);
        }

        /// <summary>
        /// Stores separate AVX vectors back into Vector3 array
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void StoreVector3Components(Vector3* output, Vector256<float> x, Vector256<float> y, Vector256<float> z)
        {
            // Extract individual components and store as Vector3s
            var xElements = x.AsVector256().GetElement(0);
            var yElements = y.AsVector256().GetElement(0);  
            var zElements = z.AsVector256().GetElement(0);

            for (int i = 0; i < 8; i++)
            {
                output[i] = new Vector3(
                    x.GetElement(i),
                    y.GetElement(i), 
                    z.GetElement(i)
                );
            }
        }

        /// <summary>
        /// Loads Vector3 components into separate SSE vectors for processing 4 vectors at once
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void LoadVector3Components128(Vector3* input, out Vector128<float> x, out Vector128<float> y, out Vector128<float> z)
        {
            var v0 = input[0]; var v1 = input[1]; var v2 = input[2]; var v3 = input[3];

            x = Vector128.Create(v0.X, v1.X, v2.X, v3.X);
            y = Vector128.Create(v0.Y, v1.Y, v2.Y, v3.Y);
            z = Vector128.Create(v0.Z, v1.Z, v2.Z, v3.Z);
        }

        /// <summary>
        /// Stores separate SSE vectors back into Vector3 array
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void StoreVector3Components128(Vector3* output, Vector128<float> x, Vector128<float> y, Vector128<float> z)
        {
            for (int i = 0; i < 4; i++)
            {
                output[i] = new Vector3(
                    x.GetElement(i),
                    y.GetElement(i),
                    z.GetElement(i)
                );
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

            if (Vector256.IsHardwareAccelerated && vectorsA.Length >= 8)
            {
                DotProductBatchAVX(vectorsA, vectorsB, results);
            }
            else if (Vector128.IsHardwareAccelerated && vectorsA.Length >= 4)
            {
                DotProductBatchSSE(vectorsA, vectorsB, results);
            }
            else
            {
                DotProductBatchScalar(vectorsA, vectorsB, results);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void DotProductBatchAVX(ReadOnlySpan<Vector3> vectorsA, ReadOnlySpan<Vector3> vectorsB, Span<float> results)
        {
            int i = 0;
            int vectorizedLength = vectorsA.Length & ~7;

            fixed (Vector3* pA = vectorsA)
            fixed (Vector3* pB = vectorsB)
            fixed (float* pResults = results)
            {
                for (; i < vectorizedLength; i += 8)
                {
                    Vector256<float> ax, ay, az, bx, by, bz;
                    LoadVector3Components(pA + i, out ax, out ay, out az);
                    LoadVector3Components(pB + i, out bx, out by, out bz);

                    // Compute dot products: ax*bx + ay*by + az*bz
                    var dots = Fma.MultiplyAdd(ax, bx, Fma.MultiplyAdd(ay, by, Vector256.Multiply(az, bz)));
                    
                    // Store results
                    for (int j = 0; j < 8; j++)
                    {
                        pResults[i + j] = dots.GetElement(j);
                    }
                }
            }

            // Handle remaining elements
            for (; i < vectorsA.Length; i++)
            {
                results[i] = Vector3.Dot(vectorsA[i], vectorsB[i]);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void DotProductBatchSSE(ReadOnlySpan<Vector3> vectorsA, ReadOnlySpan<Vector3> vectorsB, Span<float> results)
        {
            int i = 0;
            int vectorizedLength = vectorsA.Length & ~3;

            fixed (Vector3* pA = vectorsA)
            fixed (Vector3* pB = vectorsB)
            fixed (float* pResults = results)
            {
                for (; i < vectorizedLength; i += 4)
                {
                    Vector128<float> ax, ay, az, bx, by, bz;
                    LoadVector3Components128(pA + i, out ax, out ay, out az);
                    LoadVector3Components128(pB + i, out bx, out by, out bz);

                    // Compute dot products: ax*bx + ay*by + az*bz
                    var dots = Fma.MultiplyAdd(ax, bx, Fma.MultiplyAdd(ay, by, Vector128.Multiply(az, bz)));
                    
                    // Store results
                    for (int j = 0; j < 4; j++)
                    {
                        pResults[i + j] = dots.GetElement(j);
                    }
                }
            }

            // Handle remaining elements
            for (; i < vectorsA.Length; i++)
            {
                results[i] = Vector3.Dot(vectorsA[i], vectorsB[i]);
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

        /// <summary>
        /// Computes length squared for multiple Vector3s simultaneously
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void LengthSquaredBatch(ReadOnlySpan<Vector3> vectors, Span<float> results)
        {
            DotProductBatch(vectors, vectors, results);
        }

        /// <summary>
        /// Normalizes multiple Vector3s simultaneously with SIMD optimizations
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void NormalizeBatch(ReadOnlySpan<Vector3> input, Span<Vector3> output)
        {
            if (input.Length != output.Length)
                throw new ArgumentException("Input and output spans must have the same length");

            // First compute all length squared values
            Span<float> lengthsSquared = stackalloc float[Math.Min(input.Length, 256)]; // Stack-allocate for small batches
            bool useHeap = input.Length > 256;
            float[] heapLengths = useHeap ? new float[input.Length] : null;
            Span<float> lengths = useHeap ? heapLengths.AsSpan() : lengthsSquared.Slice(0, input.Length);

            LengthSquaredBatch(input, lengths);

            // Convert to reciprocal square roots and multiply
            if (Vector256.IsHardwareAccelerated && input.Length >= 8)
            {
                NormalizeBatchAVX(input, output, lengths);
            }
            else if (Vector128.IsHardwareAccelerated && input.Length >= 4)
            {
                NormalizeBatchSSE(input, output, lengths);
            }
            else
            {
                NormalizeBatchScalar(input, output, lengths);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void NormalizeBatchAVX(ReadOnlySpan<Vector3> input, Span<Vector3> output, Span<float> lengthsSquared)
        {
            int i = 0;
            int vectorizedLength = input.Length & ~7;

            fixed (Vector3* pInput = input)
            fixed (Vector3* pOutput = output)
            fixed (float* pLengths = lengthsSquared)
            {
                for (; i < vectorizedLength; i += 8)
                {
                    // Load lengths and compute reciprocal square roots
                    var lengths = Vector256.Load(pLengths + i);
                    var rsqrts = Vector256.Divide(Vector256.Create(1.0f), Avx.Sqrt(lengths));

                    // Load vector components
                    Vector256<float> x, y, z;
                    LoadVector3Components(pInput + i, out x, out y, out z);

                    // Normalize by multiplying with reciprocal square root
                    var normalizedX = Vector256.Multiply(x, rsqrts);
                    var normalizedY = Vector256.Multiply(y, rsqrts);
                    var normalizedZ = Vector256.Multiply(z, rsqrts);

                    // Store results
                    StoreVector3Components(pOutput + i, normalizedX, normalizedY, normalizedZ);
                }
            }

            // Handle remaining elements
            for (; i < input.Length; i++)
            {
                var length = MathF.Sqrt(lengthsSquared[i]);
                output[i] = length > PhysicsGlobals.EPSILON ? input[i] / length : Vector3.Zero;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void NormalizeBatchSSE(ReadOnlySpan<Vector3> input, Span<Vector3> output, Span<float> lengthsSquared)
        {
            int i = 0;
            int vectorizedLength = input.Length & ~3;

            fixed (Vector3* pInput = input)
            fixed (Vector3* pOutput = output)
            fixed (float* pLengths = lengthsSquared)
            {
                for (; i < vectorizedLength; i += 4)
                {
                    // Load lengths and compute reciprocal square roots
                    var lengths = Vector128.Load(pLengths + i);
                    var rsqrts = Vector128.Divide(Vector128.Create(1.0f), Sse.Sqrt(lengths));

                    // Load vector components
                    Vector128<float> x, y, z;
                    LoadVector3Components128(pInput + i, out x, out y, out z);

                    // Normalize by multiplying with reciprocal square root
                    var normalizedX = Vector128.Multiply(x, rsqrts);
                    var normalizedY = Vector128.Multiply(y, rsqrts);
                    var normalizedZ = Vector128.Multiply(z, rsqrts);

                    // Store results
                    StoreVector3Components128(pOutput + i, normalizedX, normalizedY, normalizedZ);
                }
            }

            // Handle remaining elements
            for (; i < input.Length; i++)
            {
                var length = MathF.Sqrt(lengthsSquared[i]);
                output[i] = length > PhysicsGlobals.EPSILON ? input[i] / length : Vector3.Zero;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void NormalizeBatchScalar(ReadOnlySpan<Vector3> input, Span<Vector3> output, Span<float> lengthsSquared)
        {
            for (int i = 0; i < input.Length; i++)
            {
                var length = MathF.Sqrt(lengthsSquared[i]);
                output[i] = length > PhysicsGlobals.EPSILON ? input[i] / length : Vector3.Zero;
            }
        }

        /// <summary>
        /// Performs linear interpolation between multiple Vector3 pairs simultaneously
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void LerpBatch(ReadOnlySpan<Vector3> from, ReadOnlySpan<Vector3> to, ReadOnlySpan<float> t, Span<Vector3> results)
        {
            if (from.Length != to.Length || from.Length != t.Length || from.Length != results.Length)
                throw new ArgumentException("All spans must have the same length");

            if (Vector256.IsHardwareAccelerated && from.Length >= 8)
            {
                LerpBatchAVX(from, to, t, results);
            }
            else if (Vector128.IsHardwareAccelerated && from.Length >= 4)
            {
                LerpBatchSSE(from, to, t, results);
            }
            else
            {
                LerpBatchScalar(from, to, t, results);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void LerpBatchAVX(ReadOnlySpan<Vector3> from, ReadOnlySpan<Vector3> to, ReadOnlySpan<float> t, Span<Vector3> results)
        {
            int i = 0;
            int vectorizedLength = from.Length & ~7;

            fixed (Vector3* pFrom = from)
            fixed (Vector3* pTo = to)
            fixed (Vector3* pResults = results)
            fixed (float* pT = t)
            {
                for (; i < vectorizedLength; i += 8)
                {
                    // Load interpolation factors
                    var tValues = Vector256.Load(pT + i);

                    // Load from and to components
                    Vector256<float> fromX, fromY, fromZ, toX, toY, toZ;
                    LoadVector3Components(pFrom + i, out fromX, out fromY, out fromZ);
                    LoadVector3Components(pTo + i, out toX, out toY, out toZ);

                    // Compute lerp: from + t * (to - from)
                    var diffX = Vector256.Subtract(toX, fromX);
                    var diffY = Vector256.Subtract(toY, fromY);
                    var diffZ = Vector256.Subtract(toZ, fromZ);

                    var resultX = Fma.MultiplyAdd(tValues, diffX, fromX);
                    var resultY = Fma.MultiplyAdd(tValues, diffY, fromY);
                    var resultZ = Fma.MultiplyAdd(tValues, diffZ, fromZ);

                    // Store results
                    StoreVector3Components(pResults + i, resultX, resultY, resultZ);
                }
            }

            // Handle remaining elements
            for (; i < from.Length; i++)
            {
                results[i] = Vector3.Lerp(from[i], to[i], t[i]);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void LerpBatchSSE(ReadOnlySpan<Vector3> from, ReadOnlySpan<Vector3> to, ReadOnlySpan<float> t, Span<Vector3> results)
        {
            int i = 0;
            int vectorizedLength = from.Length & ~3;

            fixed (Vector3* pFrom = from)
            fixed (Vector3* pTo = to)
            fixed (Vector3* pResults = results)
            fixed (float* pT = t)
            {
                for (; i < vectorizedLength; i += 4)
                {
                    // Load interpolation factors
                    var tValues = Vector128.Load(pT + i);

                    // Load from and to components
                    Vector128<float> fromX, fromY, fromZ, toX, toY, toZ;
                    LoadVector3Components128(pFrom + i, out fromX, out fromY, out fromZ);
                    LoadVector3Components128(pTo + i, out toX, out toY, out toZ);

                    // Compute lerp: from + t * (to - from)
                    var diffX = Vector128.Subtract(toX, fromX);
                    var diffY = Vector128.Subtract(toY, fromY);
                    var diffZ = Vector128.Subtract(toZ, fromZ);

                    var resultX = Fma.MultiplyAdd(tValues, diffX, fromX);
                    var resultY = Fma.MultiplyAdd(tValues, diffY, fromY);
                    var resultZ = Fma.MultiplyAdd(tValues, diffZ, fromZ);

                    // Store results
                    StoreVector3Components128(pResults + i, resultX, resultY, resultZ);
                }
            }

            // Handle remaining elements
            for (; i < from.Length; i++)
            {
                results[i] = Vector3.Lerp(from[i], to[i], t[i]);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void LerpBatchScalar(ReadOnlySpan<Vector3> from, ReadOnlySpan<Vector3> to, ReadOnlySpan<float> t, Span<Vector3> results)
        {
            for (int i = 0; i < from.Length; i++)
            {
                results[i] = Vector3.Lerp(from[i], to[i], t[i]);
            }
        }
    }
}
