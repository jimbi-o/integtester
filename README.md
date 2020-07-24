```
// syntax only
clang -std=c++17 -I ../build/_deps/doctest-src/ -fsyntax-only custom_allocator_test.cpp
// build
cmake -S src -B build -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build && ./build/integtester
// build vs
cmake.exe -S src -B buildvs -G "Visual Studio 16 2019" -A x64 -T"ClangCL" -DCMAKE_CONFIGURATION_TYPES="Debug;RelWithDebInfo;Release" && cmake.exe --build buildvs --config RelWithDebInfo && ./buildvs/RelWithDebInfo/integtester.exe
```
