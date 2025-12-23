# =============================================================================
# Comprehensive Demo Examples
#
# These demos showcase Vulcan's dual numeric/symbolic capabilities through
# non-trivial aerospace optimization problems.
# =============================================================================

add_executable(comprehensive_ascent comprehensive_ascent.cpp)
target_link_libraries(comprehensive_ascent PRIVATE vulcan)
target_precompile_headers(comprehensive_ascent REUSE_FROM getting_started)

add_executable(hypersonic_reentry hypersonic_reentry.cpp)
target_link_libraries(hypersonic_reentry PRIVATE vulcan)
target_precompile_headers(hypersonic_reentry REUSE_FROM getting_started)

add_executable(lunar_transfer lunar_transfer.cpp)
target_link_libraries(lunar_transfer PRIVATE vulcan)
target_precompile_headers(lunar_transfer REUSE_FROM getting_started)

add_executable(slosh_stability slosh_stability.cpp)
target_link_libraries(slosh_stability PRIVATE vulcan)
target_precompile_headers(slosh_stability REUSE_FROM getting_started)
