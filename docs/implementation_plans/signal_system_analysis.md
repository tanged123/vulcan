# Signal System Analysis for Icarus

**Context:** Analysis of signal routing architecture based on `system_design.md` v1.2

## 1. Core Concept Recap

Icarus is a DAG of Component models connected by **Signals** that route Janus Scalars (dual numeric/symbolic types). Each signal has:
- A direction: **Input** or **Output**
- A lifecycle: **Static** (constant post-init) or **Dynamic** (updated each step)
- A data type (currently assumed to be `double`)

---

## 2. Static vs Dynamic Signals

### Proposed Semantics

| Property | Static | Dynamic |
|:---------|:-------|:--------|
| Set when | Initialization / scenario load | Every `Step()` or conditionally |
| Mutable at runtime | No (or only via explicit reconfiguration) | Yes |
| Examples | `mass.dry_mass`, `aero.reference_area`, `config.num_fins` | `nav.altitude`, `gnc.fin_cmd`, `aero.mach` |
| Telemetry streaming | Sent once in schema, not in 60Hz packets | Included in every telemetry packet |

### Benefits of Explicit Static/Dynamic Distinction

1. **Telemetry Optimization:** Static signals don't need to consume bandwidth in the 60Hz UDP stream. Send them in the schema handshake or on-demand.

2. **DAG Evaluation Optimization:** If a component's inputs are all static, its outputs can be cached rather than recomputed each `Step()`.

3. **Dispersion Clarity (Hydra):** Monte Carlo dispersions typically apply to static signals (initial conditions, configuration). Marking them explicitly makes the campaign definition cleaner.

4. **Debugging:** Clearly separating "what changes" from "what's fixed" aids introspection.

### Implementation Sketch

```cpp
enum class SignalLifecycle { Static, Dynamic };

struct SignalDescriptor {
    std::string name;
    SignalDirection direction;  // Input, Output
    SignalLifecycle lifecycle;  // Static, Dynamic
    SignalType type;            // Double, Int, Bool (see below)
    size_t offset;              // Byte offset in state buffer
};
```

---

## 3. Data Type Support: Beyond `double`

### Current State

The Icarus Protocol (Section 6) specifies:
```
data: [N*8 bytes] double[]  — Signal values per signal_map offsets
```

This implies all signals are 64-bit floats. For a pure physics simulation, this is often sufficient—but it has limitations.

### The Case for Integer Support

**Yes, integers should be supported.** Reasons:

| Use Case | Why Integer? |
|:---------|:-------------|
| **Flight Phases / Modes** | `gnc.phase = 3` (boost, coast, terminal, etc.) is naturally discrete. |
| **Event Counters** | `prop.stage_sep_count`, `gnc.waypoint_index` |
| **Enumerated Configs** | `config.control_mode = {0: rate, 1: attitude, 2: guidance}` |
| **HIL Discrete I/O** | Real hardware often has integer command registers. |
| **Bit-Identical Determinism** | Floating-point comparisons are fragile. Integer states are exact. |

**Recommended:** Support `int32` and `int64`. The 32-bit variant covers most cases; 64-bit for timestamps or large counters.

### The Case for Boolean Support

**Debatable.** Booleans can always be represented as `int32` (0/1). However:

**Arguments for native `bool`:**
- Semantic clarity: `event.booster_sep = true` reads better than `= 1`.
- Packing efficiency: 8 bools could fit in 1 byte (though alignment may negate this).
- HIL digital I/O often thinks in terms of "on/off" signals.

**Arguments against:**
- Adds type complexity for minimal gain.
- `int32` with 0/1 convention is unambiguous and widely understood.
- The protocol already uses fixed 8-byte slots; a bool would waste 7 bytes or require packing logic.

**Recommendation:** Defer native `bool` for now. Use `int32` with a convention (`0 = false, non-zero = true`). The schema can annotate semantics:

```json
{
  "event.booster_sep": {
    "offset": 64,
    "type": "int32",
    "semantic": "boolean"
  }
}
```

This allows future tooling (Daedalus, Hydra) to render toggles/checkboxes without complicating the wire format.

---

## 4. Proposed Type System

| Type | Size | Use Case |
|:-----|:-----|:---------|
| `double` | 8 bytes | Physical quantities, continuous signals |
| `int32` | 4 bytes | Modes, phases, counters, boolean-like flags |
| `int64` | 8 bytes | Large counters, timestamps, IDs |

### Wire Format Implications

The current protocol uses fixed 8-byte slots. Options:

**Option A: Keep 8-byte alignment**
- `double` → 8 bytes
- `int32` → 4 bytes + 4 padding
- `int64` → 8 bytes

Pro: Simple. Con: Wastes space for int32.

**Option B: Packed with alignment**
- Pack signals by type, maintain natural alignment.
- Schema specifies exact byte offsets.

Pro: Efficient. Con: More complex packing logic.

**Recommendation:** Start with Option A for simplicity. The bandwidth savings from Option B are marginal at 60Hz with typical signal counts (<500 signals × 8 bytes = 4KB/packet, well under UDP limits).

---

## 5. Signal Metadata Schema Extension

Extend the SCHEMA message to carry type and lifecycle:

```json
{
  "signal_map": {
    "nav.altitude": {
      "offset": 0,
      "type": "double",
      "lifecycle": "dynamic",
      "unit": "m"
    },
    "gnc.phase": {
      "offset": 8,
      "type": "int32",
      "lifecycle": "dynamic",
      "semantic": "enum",
      "enum_values": {"0": "init", "1": "boost", "2": "coast", "3": "terminal"}
    },
    "mass.dry_mass": {
      "offset": 16,
      "type": "double",
      "lifecycle": "static",
      "unit": "kg"
    }
  }
}
```

This enables:
- Daedalus to render appropriate widgets (sliders for doubles, dropdowns for enums).
- Hydra to validate dispersion definitions against types.
- Recording/playback to interpret binary data correctly.

---

## 6. Static Signal Handling in Telemetry

### Problem

Streaming static signals at 60Hz wastes bandwidth and obscures what's actually changing.

### Proposed Solution

1. **Split the signal map** into `static_signals` and `dynamic_signals`.
2. **SCHEMA message** includes current values of all static signals.
3. **UDP telemetry** only carries dynamic signals.
4. **On reconfiguration** (e.g., scenario reload), Hermes sends an updated SCHEMA.

Alternatively, use a `dirty` bit in the packet header indicating static signals changed, prompting a schema re-fetch.

---

## 7. Summary of Recommendations

| Topic | Recommendation |
|:------|:---------------|
| Static vs Dynamic | Explicitly annotate in signal metadata. Stream only dynamic signals. |
| Integer support | Yes. Add `int32` and `int64` types. |
| Boolean support | No native type. Use `int32` with `"semantic": "boolean"` annotation. |
| Wire format | 8-byte aligned slots for simplicity. Revisit if bandwidth becomes an issue. |
| Schema extension | Add `type`, `lifecycle`, `unit`, `semantic`, `enum_values` fields. |

---

## 8. Open Questions

1. **String signals?** Unlikely needed for physics, but scenario names or error messages might want them. Probably out of scope for the core telemetry path.

2. **Array signals?** E.g., `aero.Cp[3]` for a 3-element vector. Currently each element would be a separate signal. Should arrays be first-class?

3. **Conditional dynamics?** Some signals only update when relevant (e.g., `prop.thrust` only during burn). Worth optimizing, or just send zeros?

4. **Signal groups?** For organization (all `nav.*` signals, all `aero.*` signals). Pure metadata, no runtime impact.

---

*Document generated for design discussion. Feedback welcome.*
