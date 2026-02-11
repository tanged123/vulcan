// Vulcan Frame Tree Node
// Structural metadata for a frame in the tree
#pragma once

#include <vulcan/coordinates/FrameID.hpp>

#include <string>
#include <utility>

namespace vulcan {

/// Maximum supported frame tree depth.
inline constexpr int MAX_FRAME_DEPTH = 32;

/// A node in the frame tree (setup-time structure only).
struct FrameNode {
    FrameID id;
    FrameID parent_id;
    std::string name;
    int depth;
    bool is_root;

    FrameNode()
        : id(FRAME_ECI), parent_id(FRAME_ECI), name("ECI"), depth(0),
          is_root(true) {}

    FrameNode(FrameID id_, FrameID parent_id_, std::string name_, int depth_)
        : id(id_), parent_id(parent_id_), name(std::move(name_)), depth(depth_),
          is_root(false) {}
};

} // namespace vulcan
