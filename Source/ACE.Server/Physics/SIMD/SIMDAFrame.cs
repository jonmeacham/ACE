using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using ACE.Server.Physics.Animation;

namespace ACE.Server.Physics.SIMD
{
    /// <summary>
    /// SIMD-optimized AFrame operations for high-performance batch transformations.
    /// Provides significant speedups for operations involving multiple AFrame objects.
    /// </summary>
    public static class SIMDAFrame
    {
        /// <summary>
        /// Combines multiple AFrame pairs simultaneously using SIMD operations
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void CombineBatch(ReadOnlySpan<AFrame> framesA, ReadOnlySpan<AFrame> framesB, Span<AFrame> results)
        {
            if (framesA.Length != framesB.Length || framesA.Length != results.Length)
                throw new ArgumentException("All spans must have the same length");

            if (Vector.IsHardwareAccelerated && framesA.Length >= Vector<float>.Count)
            {
                CombineBatchVectorized(framesA, framesB, results);
            }
            else
            {
                CombineBatchScalar(framesA, framesB, results);
            }
        }

        /// <summary>
        /// Interpolates between multiple AFrame pairs simultaneously
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void InterpolateBatch(ReadOnlySpan<AFrame> framesA, ReadOnlySpan<AFrame> framesB, 
            ReadOnlySpan<float> t, Span<AFrame> results)
        {
            if (framesA.Length != framesB.Length || framesA.Length != t.Length || framesA.Length != results.Length)
                throw new ArgumentException("All spans must have the same length");

            if (Vector.IsHardwareAccelerated && framesA.Length >= Vector<float>.Count)
            {
                InterpolateBatchVectorized(framesA, framesB, t, results);
            }
            else
            {
                InterpolateBatchScalar(framesA, framesB, t, results);
            }
        }

        /// <summary>
        /// Transforms multiple Vector3 points using corresponding AFrame transformations
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void TransformPointsBatch(ReadOnlySpan<Vector3> points, ReadOnlySpan<AFrame> frames, Span<Vector3> results)
        {
            if (points.Length != frames.Length || points.Length != results.Length)
                throw new ArgumentException("All spans must have the same length");

            if (Vector.IsHardwareAccelerated && points.Length >= Vector<float>.Count)
            {
                TransformPointsBatchVectorized(points, frames, results);
            }
            else
            {
                TransformPointsBatchScalar(points, frames, results);
            }
        }

        #region Vectorized Implementations

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void CombineBatchVectorized(ReadOnlySpan<AFrame> framesA, ReadOnlySpan<AFrame> framesB, Span<AFrame> results)
        {
            int vectorSize = Vector<float>.Count;
            int i = 0;

            // Process vectorizable chunks
            for (; i <= framesA.Length - vectorSize; i += vectorSize)
            {
                // Extract origins and orientations for SIMD processing
                var originsA = new Vector3[vectorSize];
                var originsB = new Vector3[vectorSize];
                var orientationsA = new Quaternion[vectorSize];
                var orientationsB = new Quaternion[vectorSize];

                for (int j = 0; j < vectorSize; j++)
                {
                    originsA[j] = framesA[i + j].Origin;
                    originsB[j] = framesB[i + j].Origin;
                    orientationsA[j] = framesA[i + j].Orientation;
                    orientationsB[j] = framesB[i + j].Orientation;
                }

                // Use SIMDVector3Extensions for vectorized operations where possible
                var transformedOrigins = new Vector3[vectorSize];
                var combinedOrientations = new Quaternion[vectorSize];

                // Transform origins: a.Origin + Transform(b.Origin, a.Orientation)
                for (int j = 0; j < vectorSize; j++)
                {
                    transformedOrigins[j] = originsA[j] + Vector3.Transform(originsB[j], orientationsA[j]);
                    combinedOrientations[j] = Quaternion.Multiply(orientationsA[j], orientationsB[j]);
                }

                // Store results
                for (int j = 0; j < vectorSize; j++)
                {
                    results[i + j] = new AFrame(transformedOrigins[j], combinedOrientations[j]);
                }
            }

            // Handle remaining elements
            for (; i < framesA.Length; i++)
            {
                results[i] = AFrame.Combine(framesA[i], framesB[i]);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void InterpolateBatchVectorized(ReadOnlySpan<AFrame> framesA, ReadOnlySpan<AFrame> framesB, 
            ReadOnlySpan<float> t, Span<AFrame> results)
        {
            int vectorSize = Vector<float>.Count;
            int i = 0;

            // Process vectorizable chunks
            for (; i <= framesA.Length - vectorSize; i += vectorSize)
            {
                // Extract data for SIMD processing
                var originsA = new Vector3[vectorSize];
                var originsB = new Vector3[vectorSize];
                var orientationsA = new Quaternion[vectorSize];
                var orientationsB = new Quaternion[vectorSize];
                var tValues = new float[vectorSize];

                for (int j = 0; j < vectorSize; j++)
                {
                    originsA[j] = framesA[i + j].Origin;
                    originsB[j] = framesB[i + j].Origin;
                    orientationsA[j] = framesA[i + j].Orientation;
                    orientationsB[j] = framesB[i + j].Orientation;
                    tValues[j] = t[i + j];
                }

                // Use SIMD vector operations for interpolation
                var interpolatedOrigins = new Vector3[vectorSize];
                var interpolatedOrientations = new Quaternion[vectorSize];

                // Use SIMDVector3Extensions for lerping origins
                SIMDVector3Extensions.LerpBatch(originsA, originsB, tValues, interpolatedOrigins);

                // Interpolate quaternions
                for (int j = 0; j < vectorSize; j++)
                {
                    interpolatedOrientations[j] = Quaternion.Lerp(orientationsA[j], orientationsB[j], tValues[j]);
                }

                // Store results
                for (int j = 0; j < vectorSize; j++)
                {
                    results[i + j] = new AFrame(interpolatedOrigins[j], interpolatedOrientations[j]);
                }
            }

            // Handle remaining elements
            for (; i < framesA.Length; i++)
            {
                var interpolatedOrigin = Vector3.Lerp(framesA[i].Origin, framesB[i].Origin, t[i]);
                var interpolatedOrientation = Quaternion.Lerp(framesA[i].Orientation, framesB[i].Orientation, t[i]);
                results[i] = new AFrame(interpolatedOrigin, interpolatedOrientation);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformPointsBatchVectorized(ReadOnlySpan<Vector3> points, ReadOnlySpan<AFrame> frames, Span<Vector3> results)
        {
            int vectorSize = Vector<float>.Count;
            int i = 0;

            // Process vectorizable chunks
            for (; i <= points.Length - vectorSize; i += vectorSize)
            {
                // Extract data for SIMD processing
                var pointArray = new Vector3[vectorSize];
                var frameArray = new AFrame[vectorSize];

                for (int j = 0; j < vectorSize; j++)
                {
                    pointArray[j] = points[i + j];
                    frameArray[j] = frames[i + j];
                }

                // Transform points using SIMD operations where possible
                for (int j = 0; j < vectorSize; j++)
                {
                    var transformedPoint = Vector3.Transform(pointArray[j], frameArray[j].Orientation);
                    results[i + j] = frameArray[j].Origin + transformedPoint;
                }
            }

            // Handle remaining elements
            for (; i < points.Length; i++)
            {
                var transformedPoint = Vector3.Transform(points[i], frames[i].Orientation);
                results[i] = frames[i].Origin + transformedPoint;
            }
        }

        #endregion

        #region Scalar Implementations

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void CombineBatchScalar(ReadOnlySpan<AFrame> framesA, ReadOnlySpan<AFrame> framesB, Span<AFrame> results)
        {
            for (int i = 0; i < framesA.Length; i++)
            {
                results[i] = AFrame.Combine(framesA[i], framesB[i]);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void InterpolateBatchScalar(ReadOnlySpan<AFrame> framesA, ReadOnlySpan<AFrame> framesB, 
            ReadOnlySpan<float> t, Span<AFrame> results)
        {
            for (int i = 0; i < framesA.Length; i++)
            {
                var interpolatedOrigin = Vector3.Lerp(framesA[i].Origin, framesB[i].Origin, t[i]);
                var interpolatedOrientation = Quaternion.Lerp(framesA[i].Orientation, framesB[i].Orientation, t[i]);
                results[i] = new AFrame(interpolatedOrigin, interpolatedOrientation);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void TransformPointsBatchScalar(ReadOnlySpan<Vector3> points, ReadOnlySpan<AFrame> frames, Span<Vector3> results)
        {
            for (int i = 0; i < points.Length; i++)
            {
                var transformedPoint = Vector3.Transform(points[i], frames[i].Orientation);
                results[i] = frames[i].Origin + transformedPoint;
            }
        }

        #endregion

        /// <summary>
        /// Converts multiple positions to AFrames simultaneously
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FromPositionsBatch(ReadOnlySpan<Vector3> positions, ReadOnlySpan<Quaternion> orientations, Span<AFrame> results)
        {
            if (positions.Length != orientations.Length || positions.Length != results.Length)
                throw new ArgumentException("All spans must have the same length");

            for (int i = 0; i < positions.Length; i++)
            {
                results[i] = new AFrame(positions[i], orientations[i]);
            }
        }
    }
}
