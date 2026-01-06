#define ANKERL_NANOBENCH_IMPLEMENT
#include <nanobench.h>

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"

int main(int argc, char** argv)
{
	doctest::Context context;

    // defaults
//    context.addFilter("test-case-exclude", "*math*");
    context.setOption("abort-after", 5);
    context.setOption("order-by", "name");
    context.applyCommandLine(argc, argv);

    // overrides
    context.setOption("no-breaks", true);

    return context.run();
}
