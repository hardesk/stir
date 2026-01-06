stir_lib = library('stir',
    [
        'stir/math/defs.hpp',
        'stir/math/neon.hpp',
        'stir/math/common.cpp',
        'stir/math/ref.cpp',
        'stir/math/neon.cpp',
    ])

stir_dep = declare_dependency(
    link_with: stir_lib,
    include_directories: '.'
)
