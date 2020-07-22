cmake_minimum_required(VERSION 3.15)
project(integtester
  VERSION 1.0
  DESCRIPTION "personal space for testing misc code"
  LANGUAGES CXX
)

include(FetchContent)
FetchContent_Declare(
  doctest
  GIT_REPOSITORY https://github.com/onqtam/doctest.git
  GIT_TAG        d5aa2bfb8f00b6260296a754af3a3a98d93f7b67
)
FetchContent_MakeAvailable(doctest)
FetchContent_Declare(
  foonathan_memory
  GIT_REPOSITORY https://github.com/foonathan/memory.git
  GIT_TAG        09b884c18abb314bfa678df27141a4dcad71c4ba
)
FetchContent_MakeAvailable(foonathan_memory)

add_executable(integtester)

target_compile_features(integtester PUBLIC cxx_std_17)
target_link_libraries(integtester PRIVATE doctest::doctest foonathan_memory)

target_include_directories(integtester
#  PUBLIC
#  "../../include"
  PRIVATE
  "."
  "${foonathan_memory_SOURCE_DIR}/include"
)
target_sources(integtester PRIVATE
  "test.cpp"
  "foonathan_memory_test.cpp"
)
