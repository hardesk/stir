include externals/

import LIBS = lua%liba{lua} benchmark%liba{benchmark}
incl_libs = externals/fmt/fmt \
			externals/doctest/doctest \
			externals/imgui/imgui \
			externals/spdlog/spdlog \
			externals/xsimd/xsimd \
			externals/mipp/mipp

lib{stir}: include/stir/hxx{**} src/stir/cxx{**} liba{$incl_libs} $LIBS 
	cxx.export.poptions += "-DEXPORT_STIR"
	cxx.export.poptions += "-I$src_base/src"
	cxx.export.poptions += "-I$src_base/include"
#cxx.export.loptions += "-l$out_base/
#	cxx.loptions += "-L$src_base/__eex/lib"
#	cxx.poptions += "-I$src_base/__eex/include"


#liba{stir}:
#cxx.export.poptions += "-I$src_base/include"

./ : lib{stir} tests/


