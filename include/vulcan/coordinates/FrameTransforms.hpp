// Vulcan Frame Graph Transforms
// Convenience wrappers for FrameContext-based transforms
#pragma once

#include <vulcan/coordinates/FrameContext.hpp>
#include <vulcan/coordinates/FrameID.hpp>
#include <vulcan/core/VulcanTypes.hpp>

namespace vulcan {

template <typename Scalar>
[[nodiscard]] inline Vec3<Scalar> transform(const Vec3<Scalar> &v, FrameID from,
                                            FrameID to,
                                            const FrameContext<Scalar> &ctx) {
    return ctx.transform(v, from, to);
}

template <typename Scalar>
[[nodiscard]] inline Vec3<Scalar>
transform_position(const Vec3<Scalar> &pos, FrameID from, FrameID to,
                   const FrameContext<Scalar> &ctx) {
    return ctx.transform_position(pos, from, to);
}

} // namespace vulcan
