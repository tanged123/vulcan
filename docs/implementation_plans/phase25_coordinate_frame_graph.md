# Phase 25: Coordinate Frame Graph — Extensible Transformation Engine

## Motivation & Problem Statement

The current Vulcan coordinate system uses an **ECEF hub-and-spoke** architecture: every frame stores its basis vectors in ECEF, and every inter-frame transform routes through ECEF as an intermediate. This was a sound initial design (matching the TAOS specification's "Unit Vector Projection Method") and provides O(2) transforms between any two frames.

However, it has real limitations:

1. **Unnecessary intermediate transforms**: Body-to-NED is conceptually a direct parent-child relationship, but the current implementation goes Body→ECEF→NED (2 rotations instead of 1). For deep hierarchies (Body→Wind→Stability, or spacecraft with multiple articulated payloads), this multiplies unnecessary operations.

2. **Hard-coded frame set**: Adding a new frame type (e.g., Sun-body, launch pad, sensor, gimbal) requires writing new factory methods and understanding the ECEF representation. Users cannot define custom frames without library modification.

3. **No explicit frame relationships**: The relationships between frames (NED is defined relative to ECEF, Body is defined relative to NED) are implicit in the code — the library has no data structure that captures which frames are connected to which, or how.

4. **No automatic path finding**: Users must manually compose transform chains. If you have 10 frames and need to go from frame A to frame G, you need to know the chain and code it yourself.

5. **No frame identity or validation**: There's nothing preventing you from accidentally passing a NED vector to a function expecting a Body vector.

---

## SOTA Analysis & Critical Assessment of the Digraph Approach

### What Industry Does

| System | Topology | Path Algorithm | Language |
|--------|----------|---------------|----------|
| **ROS tf2** | Tree (single root) | Common ancestor walk | C++ |
| **NASA SPICE** | Tree (rooted at J2000) | Common ancestor walk | C/Fortran |
| **Astropy** | General digraph | Dijkstra shortest path | Python |
| **Orekit** | Tree (rooted at GCRF) | Common ancestor walk | Java |

Key observation: **3 out of 4 major systems use trees, not general graphs**. Only Astropy uses a true digraph with Dijkstra, and Astropy is a Python library where the graph traversal cost is dwarfed by Python overhead. The tree approach with Lowest Common Ancestor (LCA) is the industry consensus for performance-critical systems.

### Critical Assessment of the Digraph + Shortest Path Approach

The user described a system using "digraphs to find the shortest edge to the fastest coordinate transform" that could handle "arbitrary coordinate transforms 10 layers deep or more with minimal performance impact." Let's evaluate this critically:

**The appeal**: With N frames, a full digraph with direct transforms between many pairs could find an optimal (shortest) path that avoids unnecessary intermediate rotations. For deep hierarchies, this could reduce a 10-hop chain to 2-3 hops if direct shortcuts exist.

**The reality for Vulcan (with CasADi constraints)**:

1. **Graph traversal cannot use symbolic values**. CasADi `MX` types cannot participate in `if/else`, cannot index arrays, and cannot control loop iteration. This means the graph topology and path-finding must be resolved *before* the symbolic computation graph is built. The graph is a **setup-time convenience**, not a symbolic operation.

2. **For aerospace simulation, frame hierarchies are shallow**. A typical simulation has 5-10 frames with depth 3-4 (ECI→ECEF→NED→Body→Wind). The theoretical benefit of shortest-path routing over a hub-spoke is marginal — saving at most 1-2 matrix operations.

3. **The cost of path-finding eclipses the savings**. Dijkstra on a 10-node graph is fast (~microseconds), but it's pure overhead compared to directly composing two 3x3 matrix-vector products. The breakeven point is a hierarchy with depth >5, which is rare in aerospace.

4. **General digraphs enable cycles, which create ambiguity**. If there are two paths from NED to Body (one direct, one through ECEF), which is canonical? The "shortest" may not be the most numerically stable. Trees avoid this entirely.

**The verdict**: A general digraph with Dijkstra is **over-engineered for aerospace simulation** and creates complexity without proportional benefit. However, the core *idea* — that frames should know their relationships and transforms should follow the natural hierarchy — is absolutely correct.

### The Right Architecture: Compile-Time Resolved Frame Tree

The optimal approach for Vulcan is a **frame tree** (not a general graph) that is:
- **Resolved at setup time** (before symbolic tracing)
- **Walked with a common-ancestor algorithm** (like tf2/SPICE, not Dijkstra)
- **Compatible with CasADi MX** (graph structure uses concrete IDs, only transforms are symbolic)
- **Extensible** (users register custom frames with their parent relationship)

This gives us:
- Direct parent-child transforms (Body→NED in 1 rotation instead of 2)
- Automatic multi-hop composition (arbitrary depth)
- User-extensible frame definitions
- Zero symbolic overhead (tree structure is non-symbolic)
- Conceptual alignment with industry standard (SPICE, Orekit, tf2)

---

## Architecture Design

### Core Concept: Separation of Frame Graph and Frame Data

```
┌──────────────────────────────────────────────────────────┐
│                      FrameRegistry                        │
│  (Setup-time only, non-templated, concrete IDs)           │
│                                                           │
│                        ┌─────┐                            │
│                        │ ECI │  (root, inertial)          │
│                        └──┬──┘                            │
│                           │                               │
│                      ┌────┴─────┐                         │
│                      │   ECEF   │  (child, fidelity-      │
│                      │          │   configurable edge)     │
│                      └──┬───┬───┘                         │
│                ┌────────┘   └────────┐                    │
│             ┌──┴──┐              ┌───┴───┐                │
│             │ NED │              │  ENU  │  ...more        │
│             └──┬──┘              └───────┘                │
│                │                                          │
│           ┌────┴─────┐                                    │
│           │   Body   │                                    │
│           └──┬───┬───┘                                    │
│         ┌────┘   └────┐                                   │
│      ┌──┴───┐   ┌─────┴─────┐                            │
│      │ Wind │   │ Stability │                             │
│      └──────┘   └───────────┘                             │
│                                                           │
│  Stores: parent IDs, tree structure, path cache           │
│  Operations: find_path(from, to) → vector<FrameID>       │
└──────────────────────────────────────────────────────────┘
                           │
                           ↓ path (concrete sequence of frame IDs)
┌──────────────────────────────────────────────────────┐
│              TransformChain<Scalar>                    │
│  (Symbolic-compatible, templated on Scalar)            │
│                                                       │
│  Applies: sequence of transforms along the resolved   │
│  path, each transform is a Scalar-templated function  │
│                                                       │
│  v_body = chain.transform(v_ned)                      │
│         = apply( NED→ECEF→... →Body, v_ned )          │
│                                                       │
│  ✓ CasADi MX compatible                              │
│  ✓ Loop bounds are structural (path length known)     │
│  ✓ No if/else on symbolic values                      │
└──────────────────────────────────────────────────────┘
```

### Key Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| **Graph topology** | Tree (single root) | Matches SPICE/Orekit/tf2; avoids cycle ambiguity |
| **Root frame** | ECI (inertial) | Matches SPICE (J2000) and Orekit (GCRF); Newton's laws apply directly; most physically fundamental frame |
| **ECI→ECEF edge** | Pluggable fidelity | The most critical edge in the tree; supports constant-omega, GMST, and future IAU 2006/2000A SOFA models |
| **Path algorithm** | Common ancestor walk | O(depth), no heap allocation, deterministic |
| **Frame identity** | Enum + string registry | Enum for built-in frames, string for user-defined |
| **Transform storage** | Functional (lambda/callable) | Each edge stores a `from_parent`/`to_parent` callable |
| **Scalar templating** | Only transforms are templated | Tree structure is non-templated (`double` IDs) |
| **Backward compatibility** | Full | Current `CoordinateFrame<Scalar>` API preserved unchanged |

---

## Implementation Plan

### Phase 25a: Frame Identity & Registry (Foundation)

#### 25a.1: Frame Identifier System

**File**: `include/vulcan/coordinates/FrameID.hpp`

```cpp
namespace vulcan {

/// Built-in frame identifiers
enum class BuiltinFrame : uint16_t {
    ECI = 0,      // Root frame (Earth-Centered Inertial)
    ECEF,         // Earth-Centered Earth-Fixed (child of ECI)
    NED,          // North-East-Down (parameterized by position)
    ENU,          // East-North-Up (parameterized by position)
    Body,         // Vehicle body-fixed
    Wind,         // Wind/velocity frame
    Stability,    // Stability axes
    Geocentric,   // Local geocentric horizon
    Rail,         // Launch rail frame
    CDA,          // Cross-range/Down-range/Altitude
    // Reserve space for future built-in frames
    _BuiltinCount
};

/// Universal frame identifier
/// Supports both built-in enum values and user-defined string-based IDs.
/// Lightweight value type — can be passed by value.
struct FrameID {
    uint32_t id;

    // Implicit conversion from built-in frames
    constexpr FrameID(BuiltinFrame b) : id(static_cast<uint32_t>(b)) {}

    // Explicit construction for user-defined frames
    explicit constexpr FrameID(uint32_t raw) : id(raw) {}

    constexpr bool operator==(const FrameID& other) const { return id == other.id; }
    constexpr bool operator!=(const FrameID& other) const { return id != other.id; }
    constexpr bool operator<(const FrameID& other) const { return id < other.id; }

    /// Check if this is a built-in frame
    constexpr bool is_builtin() const {
        return id < static_cast<uint32_t>(BuiltinFrame::_BuiltinCount);
    }
};

// Convenience constants
inline constexpr FrameID FRAME_ECEF = BuiltinFrame::ECEF;
inline constexpr FrameID FRAME_ECI  = BuiltinFrame::ECI;
inline constexpr FrameID FRAME_NED  = BuiltinFrame::NED;
inline constexpr FrameID FRAME_ENU  = BuiltinFrame::ENU;
inline constexpr FrameID FRAME_BODY = BuiltinFrame::Body;
inline constexpr FrameID FRAME_WIND = BuiltinFrame::Wind;

} // namespace vulcan
```

**Key aspects**:
- `FrameID` is a lightweight 4-byte value type, not a string
- Built-in frames get fast enum-based IDs
- User-defined frames get IDs from a counter (registered via the registry)
- No heap allocation, no virtual dispatch for ID comparison

#### 25a.2: Frame Tree Node

**File**: `include/vulcan/coordinates/FrameNode.hpp`

```cpp
namespace vulcan {

/// Maximum tree depth (prevents infinite loops, matches tf2's MAX_GRAPH_DEPTH)
inline constexpr int MAX_FRAME_DEPTH = 32;

/// A node in the frame tree
/// Non-templated — stores structural info only, not symbolic transforms.
struct FrameNode {
    FrameID id;                  // This frame's identity
    FrameID parent_id;           // Parent frame (ECI has parent = itself)
    std::string name;            // Human-readable name ("NED@DC", "Body")
    int depth;                   // Depth in tree (ECI = 0)
    bool is_root;                // True for ECI

    FrameNode() : id(FRAME_ECI), parent_id(FRAME_ECI),
                  name("ECI"), depth(0), is_root(true) {}

    FrameNode(FrameID id_, FrameID parent_, std::string name_, int depth_)
        : id(id_), parent_id(parent_), name(std::move(name_)),
          depth(depth_), is_root(false) {}
};

} // namespace vulcan
```

#### 25a.3: Frame Registry (Tree Manager)

**File**: `include/vulcan/coordinates/FrameRegistry.hpp`

```cpp
namespace vulcan {

/// Result of path finding: an ordered sequence of frame IDs from source to target
struct FramePath {
    std::vector<FrameID> frames;        // Sequence of frames on the path
    FrameID lca;                        // Lowest common ancestor
    int ascending_count;                // Number of frames from source up to LCA
    int descending_count;               // Number of frames from LCA down to target

    bool is_valid() const { return !frames.empty(); }
    int length() const { return static_cast<int>(frames.size()) - 1; }
};

/// Frame tree registry — manages frame topology and path finding
///
/// This is the setup-time data structure. It stores the tree topology
/// (parent-child relationships) and provides path-finding between any
/// two frames via Lowest Common Ancestor (LCA).
///
/// NOT templated on Scalar. Does not store transforms.
/// Thread-safe for concurrent reads after construction.
///
/// Example:
/// @code
///   FrameRegistry registry;
///   // Built-in frames are pre-registered
///   // Add a custom frame:
///   auto sensor_id = registry.register_frame("Sensor", FRAME_BODY);
///   auto path = registry.find_path(FRAME_NED, sensor_id);
///   // path.frames = {NED, ECEF, ..., Body, Sensor}
/// @endcode
class FrameRegistry {
public:
    FrameRegistry();

    /// Register a new user-defined frame as a child of an existing frame
    /// @param name Human-readable name
    /// @param parent_id Parent frame (must already be registered)
    /// @return FrameID for the new frame
    FrameID register_frame(const std::string& name, FrameID parent_id);

    /// Find path between two frames via LCA
    /// @param from Source frame
    /// @param to Target frame
    /// @return FramePath with ordered sequence, or invalid path if not connected
    FramePath find_path(FrameID from, FrameID to) const;

    /// Get node info for a frame
    const FrameNode& get_node(FrameID id) const;

    /// Check if a frame is registered
    bool has_frame(FrameID id) const;

    /// Get parent of a frame
    FrameID parent_of(FrameID id) const;

    /// Get all children of a frame
    std::vector<FrameID> children_of(FrameID id) const;

    /// Get the depth of a frame in the tree
    int depth_of(FrameID id) const;

    /// Get default registry with all built-in aerospace frames pre-registered
    /// Tree structure:
    ///   ECI (root, inertial)
    ///   └── ECEF                    ← fidelity-configurable edge
    ///       ├── NED
    ///       │   └── Body
    ///       │       ├── Wind
    ///       │       └── Stability
    ///       ├── ENU
    ///       ├── Geocentric
    ///       ├── Rail
    ///       └── CDA
    static FrameRegistry default_aerospace();

private:
    std::vector<FrameNode> nodes_;         // Indexed by FrameID
    uint32_t next_user_id_;                // Counter for user-defined IDs

    /// Walk from frame up to root, recording path
    std::vector<FrameID> path_to_root(FrameID id) const;
};
```

**LCA Algorithm** (same as tf2/SPICE):
```
find_path(A, B):
  1. Walk A to root: path_a = [A, parent(A), ..., ECI]
  2. Walk B to root: path_b = [B, parent(B), ..., ECI]
  3. Find LCA = first common element (compare from root end)
  4. Path = reverse(A..LCA) + forward(LCA..B)
  5. Return FramePath with ascending/descending counts
```

This is O(depth_A + depth_B), no heap allocation for small trees (can use stack arrays for depth < MAX_FRAME_DEPTH).

---

### Phase 25b: Transform Provider System

This phase creates the **symbolic-compatible** transform infrastructure. The key insight: the registry tells us *which* frames to traverse (using concrete IDs), and transform providers tell us *how* to transform between adjacent frames (using Scalar-templated operations).

#### 25b.1: Transform Provider Interface

**File**: `include/vulcan/coordinates/TransformProvider.hpp`

```cpp
namespace vulcan {

/// Interface for providing transforms between a child frame and its parent
///
/// Transform providers are the symbolic-compatible component of the frame
/// system. They are templated on Scalar and contain the actual mathematical
/// operations (rotations, translations) that transform vectors between frames.
///
/// Each provider handles ONE edge in the frame tree (child ↔ parent).
///
/// The transform is decomposed into:
///   - to_parent(v_child) → v_parent    (rotation + optional translation)
///   - from_parent(v_parent) → v_child  (inverse rotation + optional translation)
///
/// For pure rotations (most aerospace frames), the inverse is just the transpose.
///
/// @tparam Scalar  double or CasADi MX
template <typename Scalar>
struct TransformProvider {
    virtual ~TransformProvider() = default;

    /// Transform vector from child frame to parent frame (rotation only)
    [[nodiscard]] virtual Vec3<Scalar>
    to_parent(const Vec3<Scalar>& v) const = 0;

    /// Transform vector from parent frame to child frame (rotation only)
    [[nodiscard]] virtual Vec3<Scalar>
    from_parent(const Vec3<Scalar>& v) const = 0;

    /// Transform position from child to parent (includes origin offset)
    [[nodiscard]] virtual Vec3<Scalar>
    position_to_parent(const Vec3<Scalar>& pos) const {
        return to_parent(pos);  // Default: pure rotation (no offset)
    }

    /// Transform position from parent to child (includes origin offset)
    [[nodiscard]] virtual Vec3<Scalar>
    position_from_parent(const Vec3<Scalar>& pos) const {
        return from_parent(pos);  // Default: pure rotation (no offset)
    }
};

/// Transform provider based on an existing CoordinateFrame
///
/// This wraps the current CoordinateFrame<Scalar> as a transform provider,
/// enabling backward compatibility. The CoordinateFrame stores basis vectors
/// in ECEF (the parent), so to_parent/from_parent map directly to
/// to_ecef/from_ecef.
template <typename Scalar>
class CoordinateFrameProvider : public TransformProvider<Scalar> {
public:
    explicit CoordinateFrameProvider(CoordinateFrame<Scalar> frame)
        : frame_(std::move(frame)) {}

    Vec3<Scalar> to_parent(const Vec3<Scalar>& v) const override {
        return frame_.to_ecef(v);
    }

    Vec3<Scalar> from_parent(const Vec3<Scalar>& v) const override {
        return frame_.from_ecef(v);
    }

    Vec3<Scalar> position_to_parent(const Vec3<Scalar>& pos) const override {
        return frame_.position_to_ecef(pos);
    }

    Vec3<Scalar> position_from_parent(const Vec3<Scalar>& pos) const override {
        return frame_.position_from_ecef(pos);
    }

    const CoordinateFrame<Scalar>& frame() const { return frame_; }

private:
    CoordinateFrame<Scalar> frame_;
};

/// Transform provider from a DCM (rotation matrix)
///
/// For cases where the transform is most naturally expressed as a 3x3 matrix.
/// The DCM transforms from child to parent: v_parent = R * v_child
template <typename Scalar>
class DCMProvider : public TransformProvider<Scalar> {
public:
    explicit DCMProvider(Mat3<Scalar> R) : R_(std::move(R)) {}

    Vec3<Scalar> to_parent(const Vec3<Scalar>& v) const override {
        return R_ * v;
    }

    Vec3<Scalar> from_parent(const Vec3<Scalar>& v) const override {
        return R_.transpose() * v;
    }

private:
    Mat3<Scalar> R_;
};

/// Transform provider from a quaternion
template <typename Scalar>
class QuaternionProvider : public TransformProvider<Scalar> {
public:
    explicit QuaternionProvider(metis::Quaternion<Scalar> q) : q_(std::move(q)) {}

    Vec3<Scalar> to_parent(const Vec3<Scalar>& v) const override {
        return q_.rotate(v);
    }

    Vec3<Scalar> from_parent(const Vec3<Scalar>& v) const override {
        return q_.conjugate().rotate(v);
    }

private:
    metis::Quaternion<Scalar> q_;
};

} // namespace vulcan
```

#### 25b.2: Transform Chain (Path Executor)

**File**: `include/vulcan/coordinates/TransformChain.hpp`

```cpp
namespace vulcan {

/// Executes a pre-resolved transform path using registered providers
///
/// This is the symbolic-compatible path executor. The path (sequence of
/// frame IDs) was resolved at setup time by the FrameRegistry. The chain
/// applies transforms step-by-step along the path using TransformProviders.
///
/// The chain handles the ascending/descending split at the LCA:
///   - Ascending: source → ... → LCA (uses to_parent at each step)
///   - Descending: LCA → ... → target (uses from_parent at each step)
///
/// All operations are Scalar-templated and CasADi-compatible.
///
/// @tparam Scalar  double or CasADi MX
template <typename Scalar>
class TransformChain {
public:
    /// Construct from a resolved path and transform providers
    ///
    /// @param path The resolved frame path (from FrameRegistry::find_path)
    /// @param providers Map from (child_id, parent_id) → TransformProvider
    TransformChain(
        const FramePath& path,
        const std::vector<std::shared_ptr<TransformProvider<Scalar>>>& ascending_providers,
        const std::vector<std::shared_ptr<TransformProvider<Scalar>>>& descending_providers)
        : ascending_(ascending_providers),
          descending_(descending_providers) {}

    /// Transform a vector along the chain (rotation only)
    [[nodiscard]] Vec3<Scalar> transform_vector(const Vec3<Scalar>& v) const {
        Vec3<Scalar> result = v;

        // Ascending: source → LCA (apply to_parent at each step)
        for (int i = 0; i < static_cast<int>(ascending_.size()); ++i) {
            result = ascending_[i]->to_parent(result);
        }

        // Descending: LCA → target (apply from_parent at each step)
        // Note: descending providers are stored in order from LCA toward target
        for (int i = 0; i < static_cast<int>(descending_.size()); ++i) {
            result = descending_[i]->from_parent(result);
        }

        return result;
    }

    /// Transform a position along the chain (includes origin offsets)
    [[nodiscard]] Vec3<Scalar> transform_position(const Vec3<Scalar>& pos) const {
        Vec3<Scalar> result = pos;

        for (int i = 0; i < static_cast<int>(ascending_.size()); ++i) {
            result = ascending_[i]->position_to_parent(result);
        }

        for (int i = 0; i < static_cast<int>(descending_.size()); ++i) {
            result = descending_[i]->position_from_parent(result);
        }

        return result;
    }

    /// Number of transform steps
    int length() const {
        return static_cast<int>(ascending_.size() + descending_.size());
    }

private:
    std::vector<std::shared_ptr<TransformProvider<Scalar>>> ascending_;
    std::vector<std::shared_ptr<TransformProvider<Scalar>>> descending_;
};

} // namespace vulcan
```

**CasADi compatibility note**: The `for` loops have structural bounds (`ascending_.size()` and `descending_.size()` are concrete integers determined at setup time). The loop bodies contain only symbolic-compatible operations (`to_parent`, `from_parent`). This is Pattern C from the SOTA analysis — graph structure resolved before symbolic tracing, only mathematical operations involve `Scalar`.

---

### Phase 25c: Frame Context (The User-Facing API)

This is the primary user-facing class that ties together the registry, providers, and transform chains into a clean API.

#### 25c.1: FrameContext

**File**: `include/vulcan/coordinates/FrameContext.hpp`

```cpp
namespace vulcan {

/// Complete coordinate frame context for a simulation
///
/// FrameContext is the primary user-facing class for the new coordinate system.
/// It combines the frame tree (FrameRegistry) with active transform providers
/// (templated on Scalar) to enable transforms between any registered frames.
///
/// Usage pattern:
///   1. Create a FrameContext (from default or custom registry)
///   2. Set up frame states (position, attitude, time)
///   3. Transform vectors/positions between any frames
///
/// The context separates concerns:
///   - FrameRegistry: tree topology (non-templated, setup-time)
///   - TransformProviders: mathematical transforms (templated, symbolic-compatible)
///   - FrameContext: orchestrates both (templated, user-facing)
///
/// Example:
/// @code
///   // Setup
///   FrameContext<double> ctx;
///   auto rotation = ConstantOmegaRotation::from_wgs84();
///   ctx.set_ecef(rotation, t);       // ECI→ECEF edge (fidelity choice)
///   ctx.set_ned(lon, lat);            // ECEF→NED edge
///   ctx.set_body_euler(yaw, pitch, roll);  // NED→Body edge
///
///   // Transform between any frames
///   Vec3<double> v_body = ctx.transform(v_ned, FRAME_NED, FRAME_BODY);
///   Vec3<double> v_eci = ctx.transform(v_ecef, FRAME_ECEF, FRAME_ECI);
///
///   // Direct access to underlying CoordinateFrame (backward compat)
///   auto& ned_frame = ctx.frame(FRAME_NED);
/// @endcode
///
/// @tparam Scalar  double or CasADi MX
template <typename Scalar>
class FrameContext {
public:
    /// Construct with default aerospace frame tree
    FrameContext()
        : registry_(FrameRegistry::default_aerospace()) {}

    /// Construct with custom registry
    explicit FrameContext(FrameRegistry registry)
        : registry_(std::move(registry)) {}

    // =========================================================================
    // Frame Setup (call these to configure the frame state)
    // =========================================================================

    /// Set ECEF frame from a rotation angle (Earth rotation angle or GMST)
    /// This configures the ECI→ECEF edge. The angle can be symbolic.
    void set_ecef(Scalar rotation_angle);

    /// Set ECEF frame from a rotation model at a given time (numeric)
    /// Uses the model to compute the rotation angle, then stores the provider.
    void set_ecef(const EarthRotationModel& model, double t_seconds);

    /// Set NED frame at a geodetic position (child of ECEF)
    void set_ned(Scalar lon, Scalar lat);

    /// Set ENU frame at a geodetic position (child of ECEF)
    void set_enu(Scalar lon, Scalar lat);

    /// Set body frame from Euler angles relative to NED
    /// NED must be set first.
    void set_body_euler(Scalar yaw, Scalar pitch, Scalar roll);

    /// Set body frame from quaternion relative to NED
    void set_body_quaternion(const metis::Quaternion<Scalar>& q);

    /// Set a custom frame with a CoordinateFrame (backward compat)
    void set_frame(FrameID id, const CoordinateFrame<Scalar>& frame);

    /// Set a custom frame with a transform provider
    void set_provider(FrameID id,
                      std::shared_ptr<TransformProvider<Scalar>> provider);

    /// Register and set a new user-defined frame
    FrameID add_frame(const std::string& name, FrameID parent,
                      std::shared_ptr<TransformProvider<Scalar>> provider);

    // =========================================================================
    // Transform Operations
    // =========================================================================

    /// Transform a vector between any two registered frames
    [[nodiscard]] Vec3<Scalar> transform(
        const Vec3<Scalar>& v, FrameID from, FrameID to) const;

    /// Transform a position between any two registered frames
    [[nodiscard]] Vec3<Scalar> transform_position(
        const Vec3<Scalar>& pos, FrameID from, FrameID to) const;

    /// Build a reusable transform chain (for repeated transforms on same path)
    [[nodiscard]] TransformChain<Scalar> chain(FrameID from, FrameID to) const;

    // =========================================================================
    // Direct Frame Access (backward compatibility)
    // =========================================================================

    /// Get the CoordinateFrame for a frame (if set via CoordinateFrame)
    [[nodiscard]] const CoordinateFrame<Scalar>& frame(FrameID id) const;

    /// Get the underlying registry
    [[nodiscard]] const FrameRegistry& registry() const { return registry_; }

private:
    FrameRegistry registry_;

    // Provider storage: indexed by FrameID
    // We use a flat vector since FrameIDs are small integers
    std::vector<std::shared_ptr<TransformProvider<Scalar>>> providers_;

    // Cache of CoordinateFrames for backward compatibility
    std::vector<std::optional<CoordinateFrame<Scalar>>> frames_;

    /// Build transform chain from resolved path
    TransformChain<Scalar> build_chain(const FramePath& path) const;
};

} // namespace vulcan
```

#### 25c.2: Convenience Free Functions (Backward Compatible)

**File**: Additions to existing headers or a new `include/vulcan/coordinates/FrameTransforms.hpp`

```cpp
namespace vulcan {

/// Transform vector between frames using a context
/// This is the new primary API for multi-frame transforms.
template <typename Scalar>
Vec3<Scalar> transform(const Vec3<Scalar>& v, FrameID from, FrameID to,
                       const FrameContext<Scalar>& ctx) {
    return ctx.transform(v, from, to);
}

// The existing transform_vector(v, from_frame, to_frame) free function
// in CoordinateFrame.hpp is PRESERVED unchanged for backward compatibility.
// The new FrameContext-based API is an addition, not a replacement.

} // namespace vulcan
```

---

### Phase 25d: Built-in Frame Providers

Implement `TransformProvider` specializations for all existing built-in frames, wrapping the existing mathematical operations.

#### 25d.1: ECI→ECEF Provider (Fidelity-Configurable)

The ECI→ECEF edge is the **most critical transform in the entire tree**. Every frame below ECEF (NED, Body, Wind, etc.) inherits its accuracy. The provider system makes this edge pluggable across fidelity levels.

**File**: `include/vulcan/coordinates/providers/ECEFProvider.hpp`

```cpp
namespace vulcan {

/// Fidelity levels for the ECI→ECEF rotation
enum class EarthRotationFidelity {
    ConstantOmega,   // θ(t) = θ₀ + ωt           (~arcmin/day drift)
    GMST,            // IAU 1982 GMST formula     (~0.1 arcsec)
    IAU2006          // Full precession-nutation   (~mas, requires SOFA)
};

/// ECEF ↔ ECI transform provider
///
/// ECEF is a CHILD of ECI in the frame tree. This provider handles
/// the rotation between the inertial frame and the Earth-fixed frame.
///
/// The rotation model is injected via the existing EarthRotationModel
/// abstraction, making fidelity a configuration choice rather than
/// a code change.
///
/// Fidelity tiers:
///
/// | Tier | Model | Accuracy | Use Case |
/// |------|-------|----------|----------|
/// | Low | ConstantOmegaRotation | ~arcmin/day | Prototyping, short sims |
/// | Medium | GMSTRotation | ~0.1 arcsec | Multi-orbit, hours-to-days |
/// | High | IAU 2006/2000A (SOFA) | ~mas | Precise orbit determination |
///
/// The provider pre-computes sin/cos of the rotation angle at construction,
/// so the symbolic graph only contains the trig operations, not the
/// rotation model logic (which may use std:: math internally).
///
/// @tparam Scalar  double or CasADi MX
template <typename Scalar>
class ECEFProvider : public TransformProvider<Scalar> {
public:
    /// Construct from a rotation angle (simplest, most flexible)
    /// The angle is the Earth rotation angle (or GMST) at the current epoch.
    /// Can be a symbolic variable for CasADi graphs.
    explicit ECEFProvider(Scalar rotation_angle) {
        c_ = metis::cos(rotation_angle);
        s_ = metis::sin(rotation_angle);
    }

    /// Construct from a rotation model and time (numeric only)
    /// Evaluates the rotation model at the given time to get the angle,
    /// then stores sin/cos for the transform.
    ECEFProvider(const EarthRotationModel& model, double t_seconds) {
        double angle = model.gmst(t_seconds);
        if constexpr (std::is_floating_point_v<Scalar>) {
            c_ = std::cos(angle);
            s_ = std::sin(angle);
        } else {
            // For symbolic: wrap the numeric angle as a constant
            c_ = Scalar(std::cos(angle));
            s_ = Scalar(std::sin(angle));
        }
    }

    /// ECEF → ECI (to parent): R_z(-θ)
    /// v_eci = R_z(-θ) * v_ecef
    Vec3<Scalar> to_parent(const Vec3<Scalar>& v_ecef) const override {
        Vec3<Scalar> result;
        result(0) = c_ * v_ecef(0) - s_ * v_ecef(1);
        result(1) = s_ * v_ecef(0) + c_ * v_ecef(1);
        result(2) = v_ecef(2);
        return result;
    }

    /// ECI → ECEF (from parent): R_z(θ)
    /// v_ecef = R_z(θ) * v_eci
    Vec3<Scalar> from_parent(const Vec3<Scalar>& v_eci) const override {
        Vec3<Scalar> result;
        result(0) = c_ * v_eci(0) + s_ * v_eci(1);
        result(1) = -s_ * v_eci(0) + c_ * v_eci(1);
        result(2) = v_eci(2);
        return result;
    }

private:
    Scalar c_, s_;
};

/// Convenience factory functions
template <typename Scalar>
std::shared_ptr<ECEFProvider<Scalar>> make_ecef_provider(Scalar angle) {
    return std::make_shared<ECEFProvider<Scalar>>(angle);
}

template <typename Scalar>
std::shared_ptr<ECEFProvider<Scalar>> make_ecef_provider(
    const EarthRotationModel& model, double t_seconds) {
    return std::make_shared<ECEFProvider<Scalar>>(model, t_seconds);
}

} // namespace vulcan
```

**Design rationale for fidelity handling**:

The fidelity question is resolved by *which* rotation angle you pass to `ECEFProvider`:
- **Low fidelity**: `angle = omega * t` (constant omega)
- **Medium fidelity**: `angle = GMSTRotation().gmst(t)` (IAU 1982 polynomial)
- **High fidelity**: `angle = sofa_era(t)` + precession-nutation matrix (future Phase 3c)

For the high-fidelity case (IAU 2006/2000A), the transform is no longer a pure Z-rotation — it includes precession-nutation as a full 3x3 matrix. This is handled by extending `ECEFProvider` or using a `DCMProvider` with the full ITRF→GCRF matrix from SOFA:

```cpp
// Future high-fidelity provider (Phase 3c)
template <typename Scalar>
class SOFAECEFProvider : public TransformProvider<Scalar> {
    // Uses the full celestial-to-terrestrial matrix:
    // v_eci = C2T^T * v_ecef
    // where C2T = W(t) * R(t) * Q(t)
    //   Q = precession-nutation (CIP motion)
    //   R = Earth rotation (ERA)
    //   W = polar motion
    Mat3<Scalar> c2t_;  // celestial-to-terrestrial matrix
    // ...
};
```

This is a drop-in replacement — the rest of the tree is unaffected.

#### 25d.2: Other Earth Frame Providers

**File**: `include/vulcan/coordinates/providers/EarthProviders.hpp`

```cpp
namespace vulcan {

/// NED ↔ ECEF transform provider
template <typename Scalar>
class NEDProvider : public TransformProvider<Scalar> {
public:
    NEDProvider(Scalar lon, Scalar lat) {
        frame_ = CoordinateFrame<Scalar>::ned(lon, lat);
    }

    Vec3<Scalar> to_parent(const Vec3<Scalar>& v_ned) const override {
        return frame_.to_ecef(v_ned);
    }

    Vec3<Scalar> from_parent(const Vec3<Scalar>& v_ecef) const override {
        return frame_.from_ecef(v_ecef);
    }

private:
    CoordinateFrame<Scalar> frame_;
};

// Similarly: ENUProvider, GeocentricProvider, RailProvider, CDAProvider

} // namespace vulcan
```

#### 25d.2: Vehicle Frame Providers

**File**: `include/vulcan/coordinates/providers/VehicleProviders.hpp`

```cpp
namespace vulcan {

/// Body ↔ NED transform provider (via quaternion)
template <typename Scalar>
class BodyProvider : public TransformProvider<Scalar> {
public:
    /// From Euler angles
    BodyProvider(Scalar yaw, Scalar pitch, Scalar roll)
        : q_(metis::Quaternion<Scalar>::from_euler(roll, pitch, yaw)) {}

    /// From quaternion (body-to-NED rotation)
    explicit BodyProvider(metis::Quaternion<Scalar> q) : q_(std::move(q)) {}

    // Body → NED (parent)
    Vec3<Scalar> to_parent(const Vec3<Scalar>& v_body) const override {
        return q_.rotate(v_body);
    }

    // NED → Body (from parent)
    Vec3<Scalar> from_parent(const Vec3<Scalar>& v_ned) const override {
        return q_.conjugate().rotate(v_ned);
    }

private:
    metis::Quaternion<Scalar> q_;
};

/// Wind ↔ Body transform provider (via alpha, beta)
template <typename Scalar>
class WindProvider : public TransformProvider<Scalar> {
public:
    WindProvider(Scalar alpha, Scalar beta) {
        // Wind axes rotation from body: first rotate by -beta about z,
        // then by alpha about y
        Scalar ca = metis::cos(alpha), sa = metis::sin(alpha);
        Scalar cb = metis::cos(beta),  sb = metis::sin(beta);

        // Build DCM: wind-to-body
        R_(0,0) = ca*cb;  R_(0,1) = -ca*sb; R_(0,2) = -sa;
        R_(1,0) = sb;     R_(1,1) = cb;      R_(1,2) = Scalar(0);
        R_(2,0) = sa*cb;  R_(2,1) = -sa*sb;  R_(2,2) = ca;
    }

    Vec3<Scalar> to_parent(const Vec3<Scalar>& v_wind) const override {
        return R_ * v_wind;
    }

    Vec3<Scalar> from_parent(const Vec3<Scalar>& v_body) const override {
        return R_.transpose() * v_body;
    }

private:
    Mat3<Scalar> R_;
};

} // namespace vulcan
```

**Key benefit**: `BodyProvider` transforms directly Body↔NED with a single quaternion rotation, instead of going Body→ECEF→NED (two rotations). This is the primary performance improvement over hub-and-spoke for the most common aerospace transform.

---

### Phase 25e: Direct Parent-Child Optimizations

The frame graph enables a critical optimization: when source and target have a direct parent-child relationship, skip the LCA walk entirely and use the single provider.

```cpp
// Inside FrameContext::transform()
template <typename Scalar>
Vec3<Scalar> FrameContext<Scalar>::transform(
    const Vec3<Scalar>& v, FrameID from, FrameID to) const {

    // Same frame — identity
    if (from == to) return v;

    // Direct parent-child — single provider (fast path)
    if (registry_.parent_of(from) == to) {
        return providers_[from.id]->to_parent(v);
    }
    if (registry_.parent_of(to) == from) {
        return providers_[to.id]->from_parent(v);
    }

    // General case — LCA path walk
    auto path = registry_.find_path(from, to);
    auto chain = build_chain(path);
    return chain.transform_vector(v);
}
```

**Performance comparison for Body→NED**:

| Approach | Operations |
|----------|-----------|
| Current hub-spoke | 2 matrix-vector products (Body→ECEF→NED) |
| New direct path | 1 quaternion rotation (Body→NED) |

For a 6DOF simulation calling this transform at every timestep, this is a meaningful improvement.

---

### Phase 25f: Tests

#### Test Structure

```
tests/coordinates/
├── test_frame_registry.cpp     # 25a: Tree structure, path finding, LCA
├── test_transform_providers.cpp # 25b: Provider implementations
├── test_frame_context.cpp      # 25c: End-to-end transforms via context
├── test_direct_transforms.cpp  # 25e: Parent-child fast path verification
└── test_symbolic_context.cpp   # Symbolic mode compatibility
```

#### Key Test Cases

**Registry tests** (non-symbolic, pure tree logic):
- Built-in tree structure matches expected hierarchy (ECI root)
- LCA of (NED, Body) = NED (direct parent-child)
- LCA of (NED, ENU) = ECEF (siblings under ECEF)
- LCA of (Body, Wind) = Body (direct parent-child)
- LCA of (ECI, ECI) = ECI (identity)
- LCA of (Body, ECI) = ECI (Body walks up through NED, ECEF to root)
- Path from Body to ECI = [Body, NED, ECEF, ECI]
- Path from NED to Body = [NED, Body] (direct parent-child)
- Depth: ECI=0, ECEF=1, NED=2, Body=3, Wind=4
- User-defined frame registration and path finding
- MAX_FRAME_DEPTH guard triggers for degenerate trees

**Provider tests** (numeric + symbolic):
- ECEFProvider matches existing `CoordinateFrame::eci()` output (inverse relationship)
- ECEFProvider round-trip across all three fidelity levels (constant-omega, GMST, future SOFA)
- NEDProvider matches existing `CoordinateFrame::ned()` output
- BodyProvider: single quaternion rotation equals `body_from_euler` result
- Round-trip: v == from_parent(to_parent(v)) for all providers

**Context tests** (numeric + symbolic):
- `ctx.transform(v, FRAME_NED, FRAME_BODY)` matches `body.from_ecef(ned.to_ecef(v))`
- `ctx.transform(v, FRAME_BODY, FRAME_ECI)` composes correctly through ECEF
- Direct parent-child path is shorter than hub-spoke
- Symbolic graph builds correctly for transform chains

**Backward compatibility tests**:
- All existing `CoordinateFrame<double>` tests still pass
- `transform_vector()` free function still works
- Existing examples compile and produce identical output

---

### Phase 25g: Updated Examples & Documentation

#### New Example: `examples/coordinates/frame_graph_demo.cpp`

```cpp
#include <vulcan/vulcan.hpp>
#include <iostream>

using namespace vulcan;

int main() {
    // === Setup: create a frame context ===
    FrameContext<double> ctx;

    // Configure the ECI→ECEF edge (Earth rotation fidelity)
    // Option A: Simple constant omega (quick prototyping)
    auto rotation = ConstantOmegaRotation::from_wgs84();
    double t = 3600.0;  // 1 hour into simulation
    ctx.set_ecef(rotation, t);

    // Option B: Higher fidelity GMST model
    // auto gmst_model = GMSTRotation();
    // ctx.set_ecef(gmst_model, t);

    // Option C: Symbolic rotation angle (for CasADi graphs)
    // ctx.set_ecef(sym("gmst"));

    // Define the position (Washington DC)
    double lon = -77.0367 * constants::angle::deg2rad;
    double lat = 38.8951 * constants::angle::deg2rad;

    // Set up local and vehicle frames
    ctx.set_ned(lon, lat);
    ctx.set_body_euler(45.0 * constants::angle::deg2rad,   // yaw
                       5.0 * constants::angle::deg2rad,    // pitch
                       10.0 * constants::angle::deg2rad);  // roll

    // === Transform between any frames ===
    Vec3<double> v_body = {100.0, 0.0, 0.0};  // 100 m/s forward

    // Body → NED (direct parent-child: 1 rotation)
    auto v_ned = ctx.transform(v_body, FRAME_BODY, FRAME_NED);

    // Body → ECI (multi-hop: Body→NED→ECEF→ECI, walks full tree)
    auto v_eci = ctx.transform(v_body, FRAME_BODY, FRAME_ECI);

    // === Custom frames ===
    // Add a sensor frame mounted on the body at a fixed offset
    auto sensor_q = metis::Quaternion<double>::from_euler(
        0.0, 10.0 * constants::angle::deg2rad, 0.0);  // pitched 10deg
    auto sensor_id = ctx.add_frame("Sensor", FRAME_BODY,
        std::make_shared<QuaternionProvider<double>>(sensor_q));

    // Sensor → NED (automatically finds: Sensor→Body→NED)
    Vec3<double> v_sensor = {1.0, 0.0, 0.0};
    auto v_ned_from_sensor = ctx.transform(v_sensor, sensor_id, FRAME_NED);

    // === Symbolic mode ===
    FrameContext<SymbolicScalar> sym_ctx;
    auto sym_gmst = sym("gmst");
    auto sym_lon = sym("lon"), sym_lat = sym("lat");
    auto sym_yaw = sym("yaw"), sym_pitch = sym("pitch"), sym_roll = sym("roll");

    sym_ctx.set_ecef(sym_gmst);            // Symbolic rotation angle
    sym_ctx.set_ned(sym_lon, sym_lat);
    sym_ctx.set_body_euler(sym_yaw, sym_pitch, sym_roll);

    Vec3<SymbolicScalar> sym_v;
    sym_v << sym("vx"), sym("vy"), sym("vz");

    // Builds symbolic expression graph for Body → ECI transform
    // (traverses Body→NED→ECEF→ECI, all edges symbolic)
    auto sym_v_eci = sym_ctx.transform(sym_v, FRAME_BODY, FRAME_ECI);

    return 0;
}
```

#### Documentation Update: `docs/user_guides/frame_graph.md`

New user guide covering:
1. Motivation: why frame graphs improve on hub-and-spoke
2. Quick start with `FrameContext`
3. Built-in frame hierarchy diagram
4. Custom frame registration
5. Symbolic mode usage
6. Performance characteristics
7. Migration guide from existing `CoordinateFrame` API

---

## Phasing & Dependencies

| Sub-phase | Scope | Effort | Dependencies |
|-----------|-------|--------|-------------|
| **25a** | FrameID, FrameNode, FrameRegistry (ECI root) | ~1 day | None |
| **25b** | TransformProvider, TransformChain | ~1 day | 25a |
| **25c** | FrameContext (user API) | ~1.5 days | 25a, 25b |
| **25d** | Built-in providers: ECEFProvider (fidelity tiers), NED, ENU, Body, Wind | ~1.5 days | 25b |
| **25e** | Direct parent-child optimizations | ~0.5 day | 25c, 25d |
| **25f** | Tests (registry + providers + context + fidelity + symbolic) | ~1.5 days | 25a-25e |
| **25g** | Examples and documentation | ~1 day | 25a-25f |

**Total estimated effort**: ~8 days

---

## Backward Compatibility Strategy

This is an **additive** change. The existing API is fully preserved:

| Existing API | Status |
|-------------|--------|
| `CoordinateFrame<Scalar>` struct | **Unchanged** |
| `CoordinateFrame::ned()`, `eci()`, etc. | **Unchanged** |
| `transform_vector(v, from, to)` | **Unchanged** |
| `body_from_euler(ned, yaw, pitch, roll)` | **Unchanged** |
| `lla_to_ecef()`, `ecef_to_lla()` | **Unchanged** |
| All existing tests | **Must continue passing** |

The new API (`FrameContext`, `FrameRegistry`, etc.) is layered on top. Users who don't need the graph can continue using the existing API unchanged. Users who want extensibility, direct parent-child transforms, or automatic path finding upgrade to `FrameContext`.

---

## What This Design Does NOT Do (and Why)

| Feature | Decision | Rationale |
|---------|----------|-----------|
| **General digraph** | Use tree instead | Avoids cycle ambiguity; matches SPICE/Orekit/tf2 |
| **Dijkstra shortest path** | Use LCA walk | O(depth) not O(V+E log V); tree has unique paths |
| **Time-varying transforms** | Not in this phase | Vulcan transforms are already parameterized by time-dependent state (rotation angle, Euler angles); time buffering (tf2-style) is a separate concern |
| **SOFA integration** | Deferred to Phase 3c | ECEFProvider is designed for it (drop-in `SOFAECEFProvider`), but SOFA C library dependency is out of scope here |
| **Compile-time type safety** | Deferred | Frame tags (`Vector3<NED>`) add API complexity; can be layered on later |
| **SE(3) Lie group formalism** | Not adopted | Overkill for rotation-only frames; adds 4x4 matrix overhead |
| **Expression templates** | Not adopted | CasADi MX is already an expression graph; redundant layer |
| **Cache invalidation** | Not implemented | Incompatible with symbolic mode; CasADi MX caching is meaningless |

---

## Summary

Phase 25 transforms Vulcan's coordinate system from a flat hub-and-spoke model to a hierarchical frame tree that:

1. **Preserves the existing API** completely (zero breaking changes)
2. **Adds a frame tree rooted at ECI** (inertial root, matching SPICE J2000 / Orekit GCRF)
3. **Makes ECI→ECEF fidelity pluggable** (constant-omega, GMST, future IAU 2006/2000A SOFA)
4. **Enables direct parent-child transforms** (Body↔NED in 1 rotation, not 2)
5. **Supports user-defined frames** (sensors, gimbals, articulated payloads)
6. **Remains fully CasADi-compatible** (graph is resolved before symbolic tracing)
7. **Follows industry consensus** (tree topology, not general graph)

The key insight: **the graph structure is a setup-time convenience, not a symbolic operation**. By separating the frame topology (non-templated tree of IDs) from the transform operations (templated providers), we get the extensibility of a graph system without compromising symbolic compatibility. The inertial ECI root and pluggable ECEF provider ensure the most critical transform in the tree — Earth rotation — can be upgraded from prototype-quality to milliarcsecond precision without touching any other code.
