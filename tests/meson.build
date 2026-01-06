doctest_dep = subproject('doctest').get_variable('doctest_dep')
fmt_dep = subproject('fmt').get_variable('fmt_dep')
glm_dep = subproject('glm').get_variable('glm_dep')

nanobench_sp = subproject('nanobench')
nanobench_dep = nanobench_sp.get_variable('nanobench_dep')

math_test = executable('math_tests',
    ['math.tests.cpp', 'math.defs.tests.cpp', 'math.common.tests.cpp'],
    dependencies: [stir_dep, doctest_dep, fmt_dep]
)

executable('bench',
  ['benchmark.cpp', 'math.benchmark.cpp'],
  dependencies: [stir_dep, nanobench_dep, doctest_dep, glm_dep]
)

test('stir math test', math_test)
