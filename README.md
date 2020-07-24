```
// syntax only
clang -std=c++17 -I ../build/_deps/doctest-src/ -fsyntax-only custom_allocator_test.cpp
// build
cmake -S src -B build -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build && ./build/integtester
```
