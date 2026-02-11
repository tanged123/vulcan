// Vulcan Frame Registry
// Tree topology and LCA path finding for coordinate frame transforms
#pragma once

#include <vulcan/coordinates/FrameNode.hpp>
#include <vulcan/core/VulcanError.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace vulcan {

/// Ordered path between frames from source to target.
struct FramePath {
    std::vector<FrameID> frames;
    FrameID lca = FRAME_ECI;
    int ascending_count = 0;
    int descending_count = 0;

    [[nodiscard]] bool is_valid() const { return !frames.empty(); }

    [[nodiscard]] int length() const {
        if (frames.empty()) {
            return -1;
        }
        return static_cast<int>(frames.size()) - 1;
    }
};

/// Setup-time frame tree registry with LCA path finding.
class FrameRegistry {
  public:
    FrameRegistry()
        : next_user_id_(static_cast<uint32_t>(BuiltinFrame::_BuiltinCount)) {
        nodes_.reserve(static_cast<size_t>(BuiltinFrame::_BuiltinCount));

        // Root
        nodes_.emplace_back();

        // Built-ins are registered in enum order for contiguous indexing.
        add_builtin(BuiltinFrame::ECEF, FRAME_ECI, "ECEF");
        add_builtin(BuiltinFrame::NED, FRAME_ECEF, "NED");
        add_builtin(BuiltinFrame::ENU, FRAME_ECEF, "ENU");
        add_builtin(BuiltinFrame::Body, FRAME_NED, "Body");
        add_builtin(BuiltinFrame::Wind, FRAME_BODY, "Wind");
        add_builtin(BuiltinFrame::Stability, FRAME_BODY, "Stability");
        add_builtin(BuiltinFrame::Geocentric, FRAME_ECEF, "Geocentric");
        add_builtin(BuiltinFrame::Rail, FRAME_ECEF, "Rail");
        add_builtin(BuiltinFrame::CDA, FRAME_ECEF, "CDA");
    }

    /// Register a user-defined frame as a child of an existing frame.
    FrameID register_frame(const std::string &name, FrameID parent_id) {
        if (name.empty()) {
            throw CoordinateError("Frame name cannot be empty");
        }
        if (!has_frame(parent_id)) {
            throw CoordinateError("Parent frame is not registered");
        }

        const int depth = depth_of(parent_id) + 1;
        if (depth > MAX_FRAME_DEPTH) {
            throw CoordinateError("Frame tree exceeded MAX_FRAME_DEPTH");
        }

        FrameID id(next_user_id_++);
        nodes_.emplace_back(id, parent_id, name, depth);
        return id;
    }

    /// Find path between two frames using lowest common ancestor.
    [[nodiscard]] FramePath find_path(FrameID from, FrameID to) const {
        if (!has_frame(from) || !has_frame(to)) {
            return {};
        }

        const std::vector<FrameID> path_a = path_to_root(from);
        const std::vector<FrameID> path_b = path_to_root(to);
        if (path_a.empty() || path_b.empty()) {
            return {};
        }

        int ia = static_cast<int>(path_a.size()) - 1;
        int ib = static_cast<int>(path_b.size()) - 1;
        FrameID lca = FRAME_ECI;
        bool found_lca = false;

        while (ia >= 0 && ib >= 0 && path_a[ia] == path_b[ib]) {
            lca = path_a[ia];
            found_lca = true;
            --ia;
            --ib;
        }

        if (!found_lca) {
            return {};
        }

        FramePath result;
        result.lca = lca;
        result.ascending_count = ia + 1;
        result.descending_count = ib + 1;

        // Source -> ... -> LCA
        for (int i = 0; i <= ia + 1; ++i) {
            result.frames.push_back(path_a[static_cast<size_t>(i)]);
        }

        // LCA child -> ... -> target
        for (int i = ib; i >= 0; --i) {
            result.frames.push_back(path_b[static_cast<size_t>(i)]);
        }

        return result;
    }

    [[nodiscard]] const FrameNode &get_node(FrameID id) const {
        if (!has_frame(id)) {
            throw CoordinateError("FrameID is not registered");
        }
        return nodes_[id.id];
    }

    [[nodiscard]] bool has_frame(FrameID id) const {
        return id.id < nodes_.size();
    }

    [[nodiscard]] FrameID parent_of(FrameID id) const {
        return get_node(id).parent_id;
    }

    [[nodiscard]] std::vector<FrameID> children_of(FrameID id) const {
        if (!has_frame(id)) {
            throw CoordinateError("FrameID is not registered");
        }

        std::vector<FrameID> children;
        children.reserve(nodes_.size());
        for (const auto &node : nodes_) {
            if (node.id == id) {
                continue;
            }
            if (node.parent_id == id) {
                children.push_back(node.id);
            }
        }
        return children;
    }

    [[nodiscard]] int depth_of(FrameID id) const { return get_node(id).depth; }

    [[nodiscard]] size_t size() const { return nodes_.size(); }

    [[nodiscard]] static FrameRegistry default_aerospace() {
        return FrameRegistry();
    }

  private:
    std::vector<FrameNode> nodes_;
    uint32_t next_user_id_ = 0;

    void add_builtin(BuiltinFrame frame, FrameID parent, const char *name) {
        FrameID id(frame);
        const auto expected = static_cast<uint32_t>(nodes_.size());
        if (id.id != expected) {
            throw CoordinateError(
                "Builtin frame enum values must be contiguous");
        }

        const int depth = get_node(parent).depth + 1;
        if (depth > MAX_FRAME_DEPTH) {
            throw CoordinateError(
                "Builtin frame tree exceeded MAX_FRAME_DEPTH");
        }

        nodes_.emplace_back(id, parent, name, depth);
    }

    [[nodiscard]] std::vector<FrameID> path_to_root(FrameID id) const {
        if (!has_frame(id)) {
            return {};
        }

        std::vector<FrameID> path;
        path.reserve(static_cast<size_t>(MAX_FRAME_DEPTH + 1));

        FrameID current = id;
        for (int i = 0; i <= MAX_FRAME_DEPTH; ++i) {
            path.push_back(current);
            const auto &node = get_node(current);
            if (node.is_root || node.parent_id == current) {
                return path;
            }
            if (!has_frame(node.parent_id)) {
                throw CoordinateError("Frame tree contains missing parent");
            }
            current = node.parent_id;
        }

        throw CoordinateError("Frame tree exceeded MAX_FRAME_DEPTH");
    }
};

} // namespace vulcan
