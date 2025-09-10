# ACE SIMD Integration Plan

## Current Status: SIMD Infrastructure Built, Integration Required

The SIMD optimization classes are complete but **not yet integrated** into the existing physics engine. Here are the required integration points to actually achieve the performance gains.

## Required Integration Points

### 1. LandblockManager.cs - Batch Physics Processing

**Current Code** (Individual Processing):
```csharp
// In TickPhysics method
foreach (var landblock in landblockGroup)
    landblock.TickPhysics(portalYearTicks, movedObjects);
```

**Required Integration**:
```csharp
// Collect physics objects from all landblocks
var allPhysicsObjects = new List<PhysicsObj>();
foreach (var landblock in landblockGroup)
    allPhysicsObjects.AddRange(landblock.GetActivePhysicsObjects());

// Use SIMD batch processing
allPhysicsObjects.UpdatePhysicsSIMD((float)portalYearTicks);
```

### 2. Landblock.cs - Object Collection for Batching

**Required New Method**:
```csharp
public IEnumerable<PhysicsObj> GetActivePhysicsObjects()
{
    return worldObjects.Values
        .Where(obj => obj.PhysicsObj?.is_active() == true)
        .Select(obj => obj.PhysicsObj);
}
```

### 3. PhysicsObj.cs - SIMD Integration Points

**Current Code**:
```csharp
public bool update_object()
{
    // Individual scalar physics processing
    UpdatePhysicsInternal((float)quantum, ref newFrame);
}
```

**Required Integration**:
```csharp
// Add SIMD batch processing capability
public static void UpdateBatch(IList<PhysicsObj> objects, float quantum)
{
    using ACE.Server.Physics.SIMD;
    SIMDPhysicsEngine.UpdatePhysicsBatch(objects, quantum);
}
```

### 4. Collision Detection Integration

**Current**: Individual collision checks in various physics classes  
**Required**: Integrate `SIMDBBox` and `SIMDPhysicsEngine.ProcessCollisionsBatch()`

### 5. WorldObject_Tick.cs - Conditional SIMD Usage

**Current Code**:
```csharp
var updated = PhysicsObj.update_object();
```

**Required Integration**:
```csharp
// Allow for both individual and batch processing
if (ShouldUseBatchProcessing())
    // Will be handled by landblock batch processing
    return HandleBatchPhysics();
else
    return PhysicsObj.update_object(); // Fallback to individual
```

## Implementation Strategy

### Phase A: Core Integration (High Impact)
1. **Modify LandblockManager** to collect and batch physics objects
2. **Add batch processing methods** to Landblock class
3. **Integrate SIMDPhysicsEngine** into the main physics tick loop

### Phase B: Collision Detection (Medium Impact)  
1. **Replace individual collision checks** with batch SIMD operations
2. **Integrate SIMDBBox** into existing collision detection pipeline
3. **Update PhysicsObj collision methods** to use SIMD when beneficial

### Phase C: Fine-tuning (Low Impact)
1. **Add configuration options** for SIMD batch sizes
2. **Implement performance monitoring** to measure real-world gains
3. **Optimize thresholds** for when to use SIMD vs scalar processing

## Estimated Integration Effort

- **Phase A**: 4-6 hours of careful integration work
- **Phase B**: 3-4 hours of collision system updates  
- **Phase C**: 2-3 hours of optimization and monitoring

## Risk Mitigation

1. **Preserve existing behavior** - All SIMD classes have scalar fallbacks
2. **Gradual rollout** - Can be enabled/disabled via configuration
3. **Validation** - Extensive benchmarking suite ensures mathematical accuracy
4. **Performance monitoring** - Built-in metrics to track real-world impact

## Files Requiring Modification

### Core Physics Integration:
- `Source/ACE.Server/Managers/LandblockManager.cs`
- `Source/ACE.Server/Entity/Landblock.cs` 
- `Source/ACE.Server/Physics/PhysicsObj.cs`
- `Source/ACE.Server/WorldObjects/WorldObject_Tick.cs`

### Collision System Integration:
- `Source/ACE.Server/Physics/Collision/BBox.cs`
- Various collision detection files in physics system

### Configuration Integration:
- `Source/ACE.Server/Managers/ConfigManager.cs` (if needed)
- Physics configuration files

## Next Steps

1. **Backup current working branch** 
2. **Start with Phase A integration** - LandblockManager batch processing
3. **Test incrementally** - Validate each integration step
4. **Measure performance improvements** using the built-in benchmarking
5. **Optimize batch sizes and thresholds** based on real-world usage

## Expected Results After Integration

Once properly integrated, the SIMD optimizations should provide:
- **2-4x speedup** in physics ticking for scenarios with multiple active objects
- **3-6x speedup** in collision detection for physics-heavy landblocks  
- **Improved server scalability** - ability to handle more concurrent players
- **Reduced CPU usage** - more efficient utilization of modern CPU SIMD capabilities

---

**Status**: Infrastructure complete, integration work required to realize performance benefits.
