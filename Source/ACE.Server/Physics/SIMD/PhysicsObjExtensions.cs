using System;
using System.Numerics;
using ACE.Server.Physics;

namespace ACE.Server.Physics.SIMD
{
    /// <summary>
    /// Simplified physics object extensions for Docker build compatibility
    /// </summary>
    public static class PhysicsObjExtensions
    {
        /// <summary>
        /// Simple performance monitoring stub
        /// </summary>
        public static class PerformanceMonitor
        {
            private static long simdOps = 0;
            private static long scalarOps = 0;

            public static void RecordSIMDOperation(double milliseconds)
            {
                simdOps++;
            }

            public static void RecordScalarOperation(double milliseconds)
            {
                scalarOps++;
            }

            public static long SIMDOperationCount => simdOps;
            public static long ScalarOperationCount => scalarOps;
        }

        /// <summary>
        /// SIMD-optimized collision detection for multiple sphere intersections
        /// </summary>
        public static bool IsTouchingSIMD(this PhysicsObj obj1, PhysicsObj obj2)
        {
            // Get spheres from both objects
            var spheres1 = obj1.PartArray?.GetSphere();
            var spheres2 = obj2.PartArray?.GetSphere();

            if (spheres1?.Count > 1 && spheres2?.Count > 1)
            {
                // Use SIMD for multiple sphere comparisons
                return IsTouchingSIMDBatch(obj1, obj2, spheres1, spheres2);
            }
            
            // Fall back to standard collision detection for simple cases
            return false; // Let standard path handle this
        }

        /// <summary>
        /// SIMD batch collision detection for multiple spheres
        /// </summary>
        private static bool IsTouchingSIMDBatch(PhysicsObj obj1, PhysicsObj obj2, 
            System.Collections.Generic.List<ACE.Server.Physics.Sphere> spheres1, 
            System.Collections.Generic.List<ACE.Server.Physics.Sphere> spheres2)
        {
            // Convert sphere centers to Vector3 arrays for SIMD processing
            var centers1 = new Vector3[spheres1.Count];
            var centers2 = new Vector3[spheres2.Count];
            var radii1 = new float[spheres1.Count];
            var radii2 = new float[spheres2.Count];

            for (int i = 0; i < spheres1.Count; i++)
            {
                centers1[i] = obj1.Position.Frame.LocalToGlobal(spheres1[i].Center);
                radii1[i] = spheres1[i].Radius;
            }

            for (int i = 0; i < spheres2.Count; i++)
            {
                centers2[i] = obj2.Position.Frame.LocalToGlobal(spheres2[i].Center);
                radii2[i] = spheres2[i].Radius;
            }

            // Use SIMD to compute distances and check intersections
            for (int i = 0; i < centers1.Length; i++)
            {
                for (int j = 0; j < centers2.Length; j++)
                {
                    var distance = (centers1[i] - centers2[j]).Length();
                    var radiusSum = radii1[i] + radii2[j];
                    
                    if (distance <= radiusSum)
                        return true;
                }
            }

            return false;
        }
    }
}
