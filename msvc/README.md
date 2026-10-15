# hsdaoh_la with Visual Studio

This project builds **hsdaoh_la.exe** with MSVC. The library (libhsdaoh.dll)
is still built with MinGW; the program links against its import library
`libhsdaoh.lib`. pthreads and getopt (not part of MSVC) come from vcpkg.

## 1. One-time setup

1. Visual Studio 2022 (or newer) with the **Desktop development with C++**
   workload. Its installer includes vcpkg (component "vcpkg package manager",
   selected by default).
2. In a **Developer Command Prompt for VS**, once:

       vcpkg integrate install

## 2. Put the hsdaoh package next to the project

Extract the hsdaoh Windows package (e.g. `hsdaoh-la-win64-final.zip`) and
rename / copy its folder to `msvc\hsdaoh-win64`, so that these exist:

    msvc\hsdaoh-win64\include\hsdaoh.h
    msvc\hsdaoh-win64\include\hsdaoh_export.h
    msvc\hsdaoh-win64\lib\libhsdaoh.lib
    msvc\hsdaoh-win64\libhsdaoh.dll
    msvc\hsdaoh-win64\libusb-1.0.dll

(To use another folder, set `HsdaohDir` in the project, or build with
`msbuild hsdaoh_la.sln /p:HsdaohDir=C:\path\to\package\`. Keep the trailing `\`.)

## 3. Build

Open `msvc\hsdaoh_la.sln`, choose **Release | x64**, Build > Build Solution.

The first build takes a few minutes: vcpkg downloads and builds pthreads4w
and getopt (from `vcpkg.json`) into `msvc\vcpkg_installed`.

Result in `msvc\x64\Release\`:

    hsdaoh_la.exe
    libhsdaoh.dll, libusb-1.0.dll      copied from hsdaoh-win64 after the build
    pthreadVC3.dll, getopt.dll         copied by vcpkg

Copy all of these together when you move the program.

Command line instead of the IDE (Developer Command Prompt):

    cd msvc
    msbuild hsdaoh_la.sln /p:Configuration=Release /p:Platform=x64

## Notes

- The project uses the **MultiByte** character set on purpose. With Unicode,
  vcpkg's getopt.h switches to its wchar_t version and the build fails.
- F5 (debug) runs `hsdaoh_la -n 1M -o test.sr` in the output folder; change it
  under Project > Properties > Debugging > Command Arguments.
- `vcpkg.json` pins the vcpkg port versions with `builtin-baseline`. The
  Visual Studio copy of vcpkg needs a baseline; to update it run
  `vcpkg x-update-baseline` in this folder.
- If you cloned vcpkg yourself (`C:\vcpkg`) and ran
  `C:\vcpkg\vcpkg.exe integrate install`, the project uses that copy instead;
  nothing else changes.
- vcpkg's getopt is LGPL and is linked as a DLL (getopt.dll), which is fine.
