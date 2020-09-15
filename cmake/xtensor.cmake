include (${CMAKE_CURRENT_LIST_DIR}/CPM.cmake)

CPMFindPackage(
    NAME xtensor
    GITHUB_REPOSITORY xtensor-stack/xtensor
    GIT_TAG master
    )

