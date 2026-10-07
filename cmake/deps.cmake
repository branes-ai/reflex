# cmake/deps.cmake — FetchContent declarations for third-party deps.
#
# Permissive licenses only (MIT / BSL / BSD / Apache-2.0). Pinned versions are
# intentional; bumps go via dedicated PRs so the release-please changelog
# records dependency churn separately from feature work.
#
# The reflex library itself (branes::reflex) has no third-party dependencies;
# everything here is for tests and benchmarks.

include(FetchContent)

# Don't re-run `git fetch` on every build.
set(FETCHCONTENT_UPDATES_DISCONNECTED ON CACHE BOOL "" FORCE)

# ── catchorg/Catch2 v3 — testing framework ──────────────────────────
# SYSTEM so its headers don't trip the warnings-as-errors test build.
if(REFLEX_BUILD_TESTS AND BUILD_TESTING)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.15.0
        GIT_SHALLOW    TRUE
        SYSTEM
    )
    FetchContent_MakeAvailable(Catch2)
    list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
endif()
