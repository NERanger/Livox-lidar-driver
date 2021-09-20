# Usually livox_sdk.h and livox_def.h are under the same directory
# so we only search the path to livox_sdk.h
find_path(LivoxSDK_INCLUDE_DIR NAMES livox_sdk.h
    PATHS
        /usr/include/
        /usr/local/include/
)

# Usually at /usr/local/lib/liblivox_sdk_static.a
find_library(LivoxSDK_LIBRARY NAMES livox_sdk_static
    PATHS
        /usr/lib/
        /usr/local/lib/
)