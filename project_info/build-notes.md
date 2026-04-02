# SolveSpace — Build Notes

> **Purpose**: Environment-specific build issues and workarounds discovered during
> chamfer/fillet development on Ubuntu 22.04.
>
> **Cross-references**: [overview.md](overview.md) | [architecture.md](architecture.md)

---

## Platform

- **OS**: Ubuntu 22.04 LTS (glibc 2.35)
- **CMake**: 3.22
- **Compiler**: GCC 11
- **Build mode**: Debug (`-DCMAKE_BUILD_TYPE=Debug`)
- **Nix packages** present in environment alongside system packages

---

## CMake Option: FORCE_VENDORED_LIBS

**Defined at**: `CMakeLists.txt:79`

```cmake
option(FORCE_VENDORED_LIBS
    "Force use of vendored (bundled) libraries instead of system libraries"
    OFF)
```

Setting `FORCE_VENDORED_LIBS=ON` causes vendored copies of zlib, libpng, FreeType,
and Cairo to be used (`CMakeLists.txt:259-270`). On Linux this is normally `OFF`
(system libraries are used instead).

**Known issue**: Using `FORCE_VENDORED_LIBS=ON` on Ubuntu 22.04 causes a
**Cairo library collision** at GUI startup — the vendored Cairo conflicts with the
system's cairo/GTK layer, resulting in an immediate crash on application launch.

**Fix**: Keep `FORCE_VENDORED_LIBS=OFF` (the default) and install the missing
system library:

```sh
sudo apt install libjson-c-dev
```

---

## FindPkgConfig.cmake Workaround (CMake 3.22)

On Ubuntu 22.04 with CMake 3.22, `find_package(PkgConfig)` may fail to locate
system `.pc` files for GTK/Cairo if the `PKG_CONFIG_PATH` is not set correctly.

**Fix**: Set `PKG_CONFIG_PATH` to include both the multi-arch and shared pkgconfig
paths before running CMake:

```sh
export PKG_CONFIG_PATH="/usr/lib/x86_64-linux-gnu/pkgconfig:/usr/share/pkgconfig"
cmake .. -DENABLE_TESTS=ON -DFORCE_VENDORED_LIBS=OFF
```

This ensures pkg-config finds `gtk+-3.0.pc`, `cairo.pc`, etc. from the system
rather than from Nix or other non-standard locations.

---

## GUI Startup Issue: libjson-c-dev

**Symptom**: The GUI binary (`build/bin/solvespace`) exits immediately on launch
with a dynamic linker error related to `libjson-c.so`.

**Root cause**: GTK3 on Ubuntu 22.04 requires `libjson-c` which may not be installed
if only the minimal GTK dev packages were installed.

**Fix**:
```sh
sudo apt install libjson-c-dev
```

After installing, rebuild with `FORCE_VENDORED_LIBS=OFF`:
```sh
cmake .. -DENABLE_TESTS=ON -DFORCE_VENDORED_LIBS=OFF
cmake --build . -j$(nproc)
```

---

## Standard Build Recipe (Ubuntu 22.04)

```sh
# Install dependencies
sudo apt install libjson-c-dev libgtk-3-dev libglib2.0-dev libcairo2-dev \
    libfontconfig1-dev libpng-dev zlib1g-dev libfreetype6-dev \
    libsigc++-2.0-dev libgtkmm-3.0-dev cmake ninja-build

# Set pkg-config path for CMake 3.22
export PKG_CONFIG_PATH="/usr/lib/x86_64-linux-gnu/pkgconfig:/usr/share/pkgconfig"

# Configure and build
mkdir -p build && cd build
cmake .. -DENABLE_TESTS=ON -DFORCE_VENDORED_LIBS=OFF -DCMAKE_BUILD_TYPE=Debug
cmake --build . -j$(nproc)

# Run tests
ctest --output-on-failure
```

---

## Test Suite

- **Build option**: `-DENABLE_TESTS=ON`
- **Test binary**: `build/bin/solvespace-testsuite`
- **Run via ctest**: `ctest --test-dir build/ --output-on-failure`
- **Chamfer/fillet tests**: `ctest --test-dir build/ -R chamfer --output-on-failure`
- **Test source**: `test/group/chamfer/test.cpp` — 12+ test cases
- **Chaining test fixture**: `test/group/chamfer/chaining_test.slvs`
